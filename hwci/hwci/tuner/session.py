"""Tune session: trial ledger, stages, finals, resume."""
from __future__ import annotations

import hashlib
import json
import math
import os
import statistics
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

from .. import metrics as metricsmod
from .. import report as reportmod
from ..config import Profile, RigConfig, load_profile, profile_to_dict
from ..runner import DEFAULT_MIN_CELL_VOLTAGE, BatteryTooLowError
from ..settings import Settings, resolve_field
from . import report as tunereport
from . import search as searchmod
from .backends import TuneBackend
from .objective import check_constraints, objective_score, startup_stats
from .minduty import (compute_min_duty_for_idle, sustain_dshot_from_rows,
                      sustain_throttle_from_rows)
from .profiles import (RAMP_TRANSIENT_MAX_CURRENT_A, high_throttle_profile,
                       min_duty_measure_profile, min_duty_verify_profile,
                       probe_profile, ramp_measure_profile, startup_profile,
                       step_profile)
from .ramp import compute_max_ramp, mech_ramp_stats
from .spec import StageSpec, TuneSpec

MANIFEST_VERSION = 1


class TunePaused(RuntimeError):
    """The session checkpointed and stopped cleanly (e.g. pack too low with
    --no-prompt); resume later with ``hwci tune --resume <dir>``."""


def _atomic_write_json(path: Path, data: dict) -> None:
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(data, indent=2, sort_keys=True))
    os.replace(tmp, path)


def _git_sha(repo_root: str) -> str | None:
    try:
        out = subprocess.run(["git", "rev-parse", "--short", "HEAD"],
                             cwd=repo_root, capture_output=True, text=True)
        return out.stdout.strip() or None
    except Exception:
        return None


def _slug(overrides: dict, kind: str) -> str:
    if kind != "trial":
        base = kind.replace("_", "-")
    elif overrides:
        base = "-".join(f"{k}_{v}" for k, v in sorted(overrides.items()))
    else:
        base = "incumbent"
    return base[:60]


def _none_if_nan(v):
    return None if (isinstance(v, float) and math.isnan(v)) else v


@dataclass
class TrialPlan:
    stage: str
    kind: str                    # trial|anchor|baseline|final_winner|...
    overrides: dict[str, int]    # relative to the BASE page
    profile: Profile
    repeat: int = 0

    def signature(self) -> dict:
        return {"stage": self.stage, "kind": self.kind,
                "overrides": self.overrides, "repeat": self.repeat,
                "profile": self.profile.name}


class Tuner:
    """Runs (or resumes) one tune session against a backend."""

    def __init__(self, spec: TuneSpec, backend: TuneBackend,
                 out_dir: str | Path, *,
                 spec_text: str | None = None,
                 battery_cells: int | None = None,
                 no_prompt: bool = False,
                 resume: bool = False,
                 config_path: str | None = None,
                 prompt_fn: Callable[[str], str] = input,
                 before_trial: Callable[[int, TrialPlan], None] | None = None,
                 log: Callable[[str], None] = print):
        self.spec = spec
        self.backend = backend
        self.out = Path(out_dir)
        self.no_prompt = no_prompt
        self.prompt_fn = prompt_fn
        self.before_trial = before_trial
        self.log = log
        self.battery_cells = (battery_cells if battery_cells is not None
                              else spec.battery_cells)
        self._offsets = spec.param_offsets()
        self._cursor = 0            # next ledger entry to try to reuse
        self.executed = 0           # trials actually run this session
        # Last FET temp from a completed trial; drives cool_only_when_hot.
        self._last_fet_temp_c: float | None = None

        if resume:
            self.manifest = json.loads((self.out / "manifest.json").read_text())
            if self.manifest.get("format_version") != MANIFEST_VERSION:
                raise TunePaused(
                    f"manifest format {self.manifest.get('format_version')} "
                    f"!= {MANIFEST_VERSION}; cannot resume")
            self.base = Settings(bytes.fromhex(self.manifest["base_blob_hex"]))
            # A crash may have left ANY trial's settings flashed; put the
            # device back on the current incumbent before continuing.
            blob = self.base.apply(self.manifest.get("incumbent") or {},
                                   self._offsets)
            self.backend.program(blob.to_bytes(),
                                 self.out / "resume_settings.bin")
        else:
            self.out.mkdir(parents=True, exist_ok=True)
            if spec_text is not None:
                (self.out / "spec.yaml").write_text(spec_text)
            self.base = Settings(self.backend.read_page())
            self.base.to_bin(self.out / "base_settings.bin")
            self.manifest = {
                "format_version": MANIFEST_VERSION,
                "spec_name": spec.name,
                "spec_sha256": hashlib.sha256(
                    (spec_text or "").encode()).hexdigest(),
                "git_sha": _git_sha(
                    getattr(backend, "rig", RigConfig()).repo_root),
                "mode": backend.mode,
                "config_path": config_path,
                "battery_cells": self.battery_cells,
                "eeprom_address": backend.eeprom_address,
                "base_blob_hex": self.base.hex(),
                "jitter_reference": None,
                "incumbent": {},
                "stages": {},
                "trials": [],
                "pack_events": [],
                "result": None,
            }
            self._save()

    # -- persistence ----------------------------------------------------
    def _save(self) -> None:
        _atomic_write_json(self.out / "manifest.json", self.manifest)

    @property
    def ledger(self) -> list[dict]:
        return self.manifest["trials"]

    @property
    def incumbent(self) -> dict[str, int]:
        return dict(self.manifest["incumbent"])

    # -- battery / thermal gates -----------------------------------------
    def _min_cell_voltage(self) -> float:
        return max(DEFAULT_MIN_CELL_VOLTAGE, self.spec.pack.min_resting_cell_v)

    def _handle_low_battery(self, err: BatteryTooLowError, index: int) -> None:
        self._save()   # checkpoint before anything else
        if self.no_prompt or not self.spec.pack.prompt_on_swap:
            raise TunePaused(
                f"pack too low before trial T{index:03d} ({err}); manifest "
                f"checkpointed - swap the pack and resume with "
                f"'hwci tune --resume {self.out}'")
        self.prompt_fn(
            f"\nPack too low before trial T{index:03d}: {err}\n"
            "Swap/charge the pack, then press Enter to continue... ")
        self.manifest["pack_events"].append({
            "trial_index": index, "event": "pack_swap",
            "reason": str(err), "time": time.time()})
        self._save()

    def _swap_count(self) -> int:
        return len(self.manifest["pack_events"])

    # -- one trial (with replay/reuse) -----------------------------------
    def _trial(self, plan: TrialPlan) -> dict:
        # Replay: skip discarded entries, reuse a completed matching entry.
        while (self._cursor < len(self.ledger)
                and self.ledger[self._cursor].get("discarded")):
            self._cursor += 1
        if self._cursor < len(self.ledger):
            entry = self.ledger[self._cursor]
            sig_match = all(entry.get(k) == v
                            for k, v in plan.signature().items())
            trial_dir = self.out / entry.get("dir", "")
            complete = ((trial_dir / "trial.json").exists()
                        and (trial_dir / "metrics.json").exists())
            if sig_match and complete:
                self._cursor += 1
                return entry
            # Diverged or incomplete: quarantine this and everything after.
            self._quarantine_from(self._cursor)
        return self._execute(plan)

    def _quarantine_from(self, idx: int) -> None:
        for entry in self.ledger[idx:]:
            d = self.out / entry.get("dir", "")
            if d.exists():
                self._quarantine_dir(d)
        del self.ledger[idx:]
        self._save()

    @staticmethod
    def _quarantine_dir(d: Path) -> None:
        target = d.with_name(d.name + ".incomplete")
        n = 1
        while target.exists():
            target = d.with_name(f"{d.name}.incomplete{n}")
            n += 1
        d.rename(target)

    def _execute(self, plan: TrialPlan) -> dict:
        index = len(self.ledger)
        if self.before_trial is not None:
            self.before_trial(index, plan)
        thermal = self.spec.thermal
        if thermal.max_start_fet_temp_c is not None:
            # Skip cool-down polling when the previous trial left the FET
            # below the start limit (or we have no reading yet and assume cold).
            need_cool = (
                not thermal.cool_only_when_hot
                or (self._last_fet_temp_c is not None
                    and self._last_fet_temp_c >= thermal.max_start_fet_temp_c))
            if need_cool:
                self.backend.wait_for_cool(thermal.max_start_fet_temp_c,
                                           thermal.cool_timeout_s)
        blob = self.base.apply(plan.overrides, self._offsets)
        rel_dir = f"trials/T{index:03d}-{_slug(plan.overrides, plan.kind)}"
        trial_dir = self.out / rel_dir
        # Quarantine leftovers from a crash mid-trial: any dir claiming this
        # trial index (whatever its slug) that never completed.
        for stale in sorted(self.out.glob(f"trials/T{index:03d}-*")):
            if stale.is_dir() and ".incomplete" not in stale.name:
                self._quarantine_dir(stale)

        rig = getattr(self.backend, "rig", RigConfig())
        meta = {
            "target": rig.target, "mode": self.backend.mode,
            "profile": plan.profile.name,
            "profile_def": profile_to_dict(plan.profile),
            "pole_pairs": rig.pole_pairs,
            "motor": rig.motor_name, "prop": rig.prop,
            "tune_stage": plan.stage, "tune_kind": plan.kind,
            "tune_overrides": plan.overrides,
            "settings_blob_sha256": blob.sha256(),
        }
        while True:
            try:
                result, extra = self.backend.run_trial(
                    blob.to_bytes(), plan.profile,
                    trial_dir / "settings.bin", meta,
                    battery_cells=self.battery_cells,
                    min_cell_voltage=self._min_cell_voltage())
                break
            except BatteryTooLowError as e:
                self._handle_low_battery(e, index)

        result.meta.update(extra)
        result.save(trial_dir)
        m = metricsmod.compute(result, plan.profile)
        (trial_dir / "metrics.json").write_text(json.dumps(m, indent=2))

        is_startup = plan.profile.name == "tune_startup"
        # Min-duty crawl/verify holds sit at DShot idle (~few hundred RPM on
        # a big prop); the "failed start" min_rpm gate is for efficiency
        # probes, not those profiles.
        is_min_duty = plan.profile.name in (
            "tune_min_duty_measure", "tune_min_duty_verify")
        st = (startup_stats(result, plan.profile,
                            self.spec.constraints.startup.min_rpm)
              if is_startup else None)
        fails = check_constraints(
            m, result.meta, self.spec.constraints,
            jitter_reference=self.manifest["jitter_reference"],
            settings_verified=bool(extra.get("settings_verified")),
            startup=st,
            min_start_rpm=(None if (is_startup or is_min_duty)
                           else self.spec.constraints.startup.min_rpm))
        score = objective_score(m, self.spec.objective) if not fails else None
        summary = m.get("summary", {})

        fet = _none_if_nan(summary.get("max_fet_temp_c"))
        entry = {
            **plan.signature(),
            "index": index,
            "dir": rel_dir,
            "blob_sha256": blob.sha256(),
            "settings_verified": bool(extra.get("settings_verified")),
            "resting_v": extra.get("resting_v"),
            "score_raw": None if score is None else round(score, 5),
            "score_norm": None,
            "disqualified": fails or None,
            "startup": st,
            "jitter_pct": summary.get("worst_zc_jitter_pct"),
            "fet_temp_c": fet,
            "discarded": False,
        }
        (trial_dir / "trial.json").write_text(
            json.dumps(entry, indent=2, sort_keys=True))
        self.ledger.append(entry)
        self._cursor = len(self.ledger)
        self.executed += 1
        if fet is not None:
            self._last_fet_temp_c = float(fet)
        self._save()
        if fails:
            verdict = f"DISQUALIFIED ({'; '.join(fails)})"
        elif score is not None:
            verdict = f"score {score:.3f} g/W"
        else:
            verdict = "ok (no scored points)"
        self.log(f"  T{index:03d} {plan.stage}/{plan.kind} "
                 f"{plan.overrides or '{}'} -> {verdict}")
        return entry

    # -- anchor-normalized scoring (delegates to search) -----------------
    @staticmethod
    def _normalize(entries: list[dict], anchors: list[tuple[int, float]],
                   positions: dict[int, int]) -> None:
        searchmod.normalize(entries, anchors, positions)

    @staticmethod
    def _drift_factor(anchors: list[tuple[int, float]], pos: int) -> float:
        return searchmod.drift_factor(anchors, pos)

    # -- candidate scoring / winner picking --------------------------------
    def _distance_to_default(self, overrides: dict[str, int]) -> float:
        dist = 0.0
        for name, v in overrides.items():
            f = resolve_field(name, self._offsets.get(name))
            span = max(1, f.hi - f.lo)
            dist += abs(int(v) - self.base.get(name, self._offsets.get(name))) \
                / span
        return dist

    def _pick_winner(self, cands: list[dict]) -> dict | None:
        return searchmod.pick_winner(
            cands, noise_floor_pct=self.spec.objective.noise_floor_pct,
            distance_fn=self._distance_to_default)

    # -- stages ------------------------------------------------------------
    def _merged(self, stage: StageSpec, overrides: dict) -> dict:
        return {**self.incumbent, **stage.fixed, **overrides}

    def _stage_profile(self, stage: StageSpec) -> Profile:
        return self._profile_by_name(stage.profile)

    def _profile_by_name(self, name: str | None) -> Profile:
        if name in (None, "tune_probe", "probe"):
            return probe_profile(self.spec)
        if name == "tune_startup":
            return startup_profile(self.spec)
        if name == "tune_step":
            return step_profile(self.spec)
        if name == "tune_high_throttle":
            thr = self.spec.finals.high_throttle or 0.70
            return high_throttle_profile(
                self.spec, thr, self.spec.finals.high_throttle_dwell_s)
        return load_profile(name)

    def _run_scored_stage(self, stage: StageSpec) -> None:
        profile = self._stage_profile(stage)
        anchors: list[tuple[int, float]] = []
        positions: dict[int, int] = {}
        entries_all: list[dict] = []
        pos = 0
        since_anchor = [0]

        def run_one(kind: str, overrides: dict, repeat: int = 0) -> dict:
            nonlocal pos
            e = self._trial(TrialPlan(stage=stage.name, kind=kind,
                                      overrides=overrides, profile=profile,
                                      repeat=repeat))
            positions[e["index"]] = pos
            entries_all.append(e)
            pos += 1
            return e

        def anchor() -> None:
            e = run_one("anchor", self.incumbent)
            anchors.append((positions[e["index"]], e.get("score_raw")))
            since_anchor[0] = 0

        def _effective_anchors_every() -> int:
            every = max(1, self.spec.anchors_every)
            if not self.spec.adaptive_anchors or len(anchors) < 2:
                return every
            # Consecutive usable anchors that drift hard -> re-anchor sooner.
            usable = [(p, s) for p, s in anchors
                      if s is not None and s > 0]
            if len(usable) < 2:
                return every
            (_, s0), (_, s1) = usable[-2], usable[-1]
            drift_pct = abs(s1 - s0) / s0 * 100.0
            if drift_pct >= self.spec.anchor_drift_pct:
                return max(1, every // 2)
            return every

        def candidate(overrides: dict, repeat: int = 0) -> dict:
            if since_anchor[0] >= _effective_anchors_every():
                anchor()
            e = run_one("trial", overrides, repeat)
            since_anchor[0] += 1
            return e

        def renorm() -> None:
            # Keep climb decisions on the same basis as final ranking:
            # re-apply anchor-normalized scores after every trial.
            searchmod.normalize(entries_all, anchors, positions)

        anchor()
        cands: list[dict] = []
        if stage.sweep:
            param = self.spec.parameters[stage.sweep]
            f = resolve_field(param.name, param.offset)
            values = list(dict.fromkeys(param.values))
            inc_val = self.base.apply(self.incumbent, self._offsets).get(
                param.name, param.offset)
            if inc_val not in values:
                values.append(inc_val)
            # Post-modes polish: only re-test a neighborhood of the incumbent.
            if stage.polish_radius is not None:
                r = stage.polish_radius
                values = [v for v in values if abs(v - inc_val) <= r]
                if inc_val not in values:
                    values.append(inc_val)
                self.log(f"stage {stage.name}: polish_radius={r} around "
                         f"{param.name}={inc_val} -> values {sorted(values)}")

            def test_value(v: int) -> dict:
                ent = [candidate(self._merged(stage, {param.name: v}), r)
                       for r in range(stage.repeats)]
                c = {"overrides": self._merged(stage, {param.name: v}),
                     "value": v, "entries": ent, "order": len(cands)}
                cands.append(c)
                renorm()
                return c

            if stage.search == "climb":
                searchmod.climb(sorted(values), inc_val, test_value)
            else:
                for v in values:
                    test_value(v)
            renorm()
            if param.refine_step and stage.polish_radius is None:
                # Refine around the ranking argmax (normalized + soft DQ),
                # not the noise-floor tie-break winner. Skip when polish
                # already limited the neighborhood.
                center = searchmod.argmax_value(cands)
                if center is not None:
                    for dv in (-param.refine_step, param.refine_step):
                        v = center + dv
                        if v in values or not f.lo <= v <= f.hi:
                            continue
                        values.append(v)
                        test_value(v)
        else:
            candidates = stage.ab_candidates or []
            ents: list[list[dict]] = [[] for _ in candidates]
            for r in range(stage.repeats):      # interleaved: c0 c1 c2 | c0..
                for i, ov in enumerate(candidates):
                    ents[i].append(candidate(self._merged(stage, ov), r))
                    renorm()
            cands = [{"overrides": self._merged(stage, ov), "raw_ov": ov,
                      "entries": ents[i], "order": i}
                     for i, ov in enumerate(candidates)]
        anchor()
        renorm()
        self._save()
        winner = self._pick_winner(cands)
        argmax = searchmod.efficiency_argmax(cands)
        reason = searchmod.winner_reason(winner, argmax)
        if winner is not None:
            self.manifest["incumbent"] = dict(winner["overrides"])
        stage_rec = {
            "winner": None if winner is None else winner["overrides"],
            "winner_score": None if winner is None else round(winner["score"], 5),
            "efficiency_argmax": (None if argmax is None
                                  else argmax["overrides"]),
            "efficiency_argmax_score": (None if argmax is None
                                        else round(argmax["score"], 5)),
            "winner_reason": reason,
            "trials": [e["index"] for e in entries_all],
        }
        self.manifest["stages"][stage.name] = stage_rec
        self._save()
        msg = f"stage {stage.name}: winner " \
              f"{None if winner is None else winner['overrides']}"
        if (argmax is not None and winner is not None
                and argmax["overrides"] != winner["overrides"]):
            msg += (f" (efficiency argmax {argmax['overrides']} @ "
                    f"{argmax['score']:.3f}; chose via {reason})")
        self.log(msg)

    def _trial_rows(self, entry: dict) -> list[dict]:
        import csv
        path = self.out / entry.get("dir", "") / "samples.csv"
        with open(path, newline="") as f:
            return list(csv.DictReader(f))

    def _ramp_fallback_candidates(self) -> list[dict]:
        """Ranked (best efficiency score first) alternative FULL settings
        combos to retry ramp certification against.

        Uses the same multi-repeat-soft metric as pick_winner (median over
        clean entries; a value with *all* entries DQ is excluded).
        """
        adv = self.spec.parameters.get("advance_level")
        adv_stage = next((s for s in self.spec.stages
                          if s.sweep == "advance_level"), None)
        seen = {frozenset(self.incumbent.items())}
        ranked: list[tuple[dict, float]] = []
        if adv is not None and adv_stage is not None:
            for v in dict.fromkeys(adv.values):
                full = {**self.incumbent, "advance_level": v}
                key = frozenset(full.items())
                if key in seen:
                    continue
                entries = [t for t in self.manifest["trials"]
                          if t.get("stage") == adv_stage.name
                          and t.get("kind") == "trial"
                          and t.get("overrides", {}).get("advance_level") == v
                          and not t.get("discarded")]
                score = searchmod.candidate_metric(entries)
                if score is None:
                    continue
                seen.add(key)
                ranked.append((full, score))
        ranked.sort(key=lambda item: item[1], reverse=True)
        out = [full for full, _ in ranked]
        if frozenset() not in seen:
            out.append({})
        return out

    def _run_measure_stage(self, stage: StageSpec) -> None:
        if stage.measure == "min_duty":
            self._run_min_duty_stage(stage)
        else:
            self._run_ramp_measure_stage(stage)

    def _run_min_duty_stage(self, stage: StageSpec) -> None:
        """DShot-idle measure of minimum_duty_cycle, then verify with raise."""
        f = resolve_field("minimum_duty_cycle", None)
        indices: list[int] = []
        # Floor the measure run so the firmware floor cannot hide the plant
        # sustain threshold (eeprom unit 1 = 0.5% duty).
        measure_ov = self._merged(stage, {"minimum_duty_cycle": f.lo})
        e = self._trial(TrialPlan(
            stage=stage.name, kind="measure",
            overrides=measure_ov,
            profile=min_duty_measure_profile(self.spec)))
        indices.append(e["index"])
        # Analyse samples even if low holds tripped demag/bemf constraints:
        # the crawl's job is to find where sustain ends, and those failures
        # are expected on the bottom rungs.
        rig = getattr(self.backend, "rig", RigConfig())
        pp = int(getattr(rig, "pole_pairs", None) or 7)
        # Kick/false-lock on this prop sits ~350-450 RPM; real idle sustain
        # under an adequate floor is typically ≥600-800 RPM. Gate above the
        # kick band so coast/kick cannot pass as "sustain".
        min_rpm = max(500.0, float(self.spec.constraints.startup.min_rpm) * 0.5)
        stats = sustain_dshot_from_rows(
            self._trial_rows(e), min_rpm=min_rpm, pole_pairs=pp)
        computed: Optional[int] = None
        if stats is None:
            self.log(f"stage {stage.name}: no sustained DShot hold "
                     f"(min_rpm {min_rpm:.0f}); trying verify search from "
                     f"mid-range minimum_duty_cycle")
            start_v = max(f.lo, (f.lo + f.hi) // 4)
        else:
            computed = compute_min_duty_for_idle(
                stats["sustain_dshot"], lo=f.lo, hi=f.hi,
                margin=stage.margin, measure_eeprom=f.lo)
            self.log(
                f"stage {stage.name}: sustain @ DShot "
                f"{stats['sustain_dshot']:.0f} "
                f"({stats['sustain_segment']}, "
                f"{stats['sustain_rpm']:.0f} rpm, "
                f"~{stats['sustain_duty_counts']:.0f}/2000 duty), "
                f"{stats['failed_holds']}/{stats['holds']} holds failed "
                f"-> minimum_duty_cycle {computed} "
                f"(margin {stage.margin}, targets DShot-48 floor)")
            start_v = computed

        def verify_with_backoff(base_ov: dict, start: int) -> Optional[int]:
            v = start
            while True:
                # Verify requires DShot 48/50/55 holds to sustain under the
                # programmed floor — not only "no demag DQ".
                ev = self._trial(TrialPlan(
                    stage=stage.name, kind="verify",
                    overrides={**base_ov, "minimum_duty_cycle": v},
                    profile=min_duty_verify_profile(self.spec)))
                indices.append(ev["index"])
                vstats = sustain_dshot_from_rows(
                    self._trial_rows(ev), min_rpm=min_rpm, pole_pairs=pp)
                ok = (not ev.get("disqualified")
                      and vstats is not None
                      and vstats["failed_holds"] == 0
                      and vstats["lowest_dshot"] <= 48.5)
                if ok:
                    return v
                nxt = min(f.hi, v + max(1, int(math.ceil(v * 0.25))))
                if nxt == v:
                    return None
                if ev.get("disqualified"):
                    why = ev["disqualified"]
                elif vstats is None:
                    why = "no sustained holds"
                else:
                    why = (f"{vstats['failed_holds']} hold(s) below min_rpm "
                           f"(lowest sustained DShot "
                           f"{vstats.get('sustain_dshot', '?')})")
                self.log(f"stage {stage.name}: verify failed at "
                         f"minimum_duty_cycle {v} ({why}); raising to {nxt}")
                v = nxt

        winner_val = verify_with_backoff(self._merged(stage, {}), start_v)
        if winner_val is not None:
            self.manifest["incumbent"] = {
                **self.incumbent, **stage.fixed,
                "minimum_duty_cycle": winner_val}
        else:
            self.log(f"stage {stage.name}: no minimum_duty_cycle in range "
                     "passed DShot-idle verify; leaving incumbent unchanged")
        measured = None
        if stats is not None:
            measured = {
                k: (round(v, 4) if isinstance(v, float) else v)
                for k, v in stats.items()
            }
        self.manifest["stages"][stage.name] = {
            "winner": (None if winner_val is None
                       else {"minimum_duty_cycle": winner_val}),
            "winner_score": None,
            "measured": measured,
            "computed_minimum_duty_cycle": computed,
            "trials": indices,
        }
        self._save()
        self.log(f"stage {stage.name}: winner "
                 f"{None if winner_val is None else {'minimum_duty_cycle': winner_val}}")

    def _run_ramp_measure_stage(self, stage: StageSpec) -> None:
        """Physics-based max_ramp: measure, compute, bubble-search verify.

        Verify no longer starts at the field ceiling and multiplies down by
        0.7 (that forced a long desync thrash on heavy props). It climbs from
        the safe floor (exponential probe + binary refine) so failed trials
        are rare and bounded.
        """
        f = resolve_field("max_ramp", None)
        indices: list[int] = []

        def measure_and_compute() -> tuple[Optional[dict], Optional[int]]:
            e = self._trial(TrialPlan(
                stage=stage.name, kind="measure",
                overrides=self._merged(stage, {"max_ramp": f.hi}),
                profile=ramp_measure_profile(self.spec)))
            indices.append(e["index"])
            if e.get("disqualified"):
                self.log(f"stage {stage.name}: measurement run disqualified "
                         f"({e['disqualified']}); falling back to bubble "
                         "search on max_ramp from the floor")
                return None, None
            stats = mech_ramp_stats(self._trial_rows(e))
            if stats is None:
                self.log(f"stage {stage.name}: no usable step response in "
                         "the measurement run; falling back to bubble "
                         "search on max_ramp from the floor")
                return None, None
            limit = ramp_measure_profile(self.spec).safety.max_current_a
            budget = max(0.75 * (limit or RAMP_TRANSIENT_MAX_CURRENT_A)
                         - stats["i_hi_a"], 5.0)
            computed = compute_max_ramp(stats, current_budget_a=budget,
                                        lo=f.lo, hi=f.hi, margin=stage.margin)
            self.log(
                f"stage {stage.name}: tau {stats['tau_ms']:.0f} ms, slew "
                f"{stats['slew_erpm_per_s']:.0f} eRPM/s, "
                f"k {stats['k_a_per_pct']:.2f} A/%, budget {budget:.1f} A "
                f"-> max_ramp {computed}")
            return stats, computed

        def try_verify(base_ov: dict, v: int) -> bool:
            ev = self._trial(TrialPlan(
                stage=stage.name, kind="verify",
                overrides={**base_ov, "max_ramp": v},
                profile=step_profile(self.spec)))
            indices.append(ev["index"])
            ok = not ev.get("disqualified")
            if ok:
                self.log(f"stage {stage.name}: max_ramp {v} PASS")
            else:
                self.log(f"stage {stage.name}: max_ramp {v} FAIL "
                         f"({ev.get('disqualified')})")
            return ok

        def verify_bubble_search(base_ov: dict,
                                 hint: Optional[int]) -> Optional[int]:
            """Largest safe max_ramp via floor-first bubble + binary refine.

            1. Prove ``f.lo`` is safe (else no winner).
            2. If ``hint`` is above the floor, try it (one shot): on pass,
               bubble up from hint; on fail, binary-search (lo, hint).
            3. Else geometric climb from the floor until first fail, then
               binary-search the gap.
            """
            if not try_verify(base_ov, f.lo):
                return None
            last_pass = f.lo
            first_fail: Optional[int] = None

            def bubble_up(start: int) -> int:
                """Geometric climb from a known-good value; returns last pass."""
                nonlocal first_fail
                cur = start
                while cur < f.hi:
                    nxt = min(f.hi, max(cur + 1, int(cur * 2)))
                    if nxt == cur:
                        break
                    if try_verify(base_ov, nxt):
                        cur = nxt
                    else:
                        first_fail = nxt
                        break
                return cur

            def binary_between(lo_ok: int, hi_fail: int) -> int:
                lo, hi = lo_ok, hi_fail
                while hi - lo > 1:
                    mid = (lo + hi) // 2
                    if try_verify(base_ov, mid):
                        lo = mid
                    else:
                        hi = mid
                return lo

            if hint is not None and f.lo < hint <= f.hi:
                if try_verify(base_ov, hint):
                    last_pass = bubble_up(hint)
                else:
                    first_fail = hint
                    return binary_between(last_pass, first_fail)
            else:
                last_pass = bubble_up(last_pass)

            if first_fail is None:
                return last_pass
            return binary_between(last_pass, first_fail)

        stats, computed = measure_and_compute()
        winner_val = verify_bubble_search(
            self._merged(stage, {}), computed)

        winner_ov: Optional[dict] = None   # None => the top incumbent won
        if winner_val is None:
            candidates = self._ramp_fallback_candidates()
            if candidates:
                self.log(f"stage {stage.name}: incumbent has no safe "
                         f"max_ramp anywhere in range; trying "
                         f"{len(candidates)} alternative setting(s) from "
                         "the coordinate search instead of keeping it")
            for cand in candidates:
                self.log(f"stage {stage.name}: retrying ramp certification "
                         f"at {cand or '(firmware defaults)'}")
                wv = verify_bubble_search({**cand, **stage.fixed}, computed)
                if wv is not None:
                    winner_val, winner_ov = wv, cand
                    self.log(f"stage {stage.name}: {cand or '(defaults)'} "
                             f"is ramp-safe (max_ramp {wv}) - overriding "
                             "the coordinate search's efficiency winner")
                    break

        if winner_val is not None:
            base = self.incumbent if winner_ov is None else winner_ov
            self.manifest["incumbent"] = {
                **base, **stage.fixed, "max_ramp": winner_val}
        elif self.incumbent:
            self.log(f"stage {stage.name}: no candidate (including "
                     "firmware defaults) is ramp-safe; resetting the "
                     "incumbent to firmware defaults for the rest of the "
                     "session")
            self.manifest["incumbent"] = {}
        self.manifest["stages"][stage.name] = {
            "winner": (None if winner_val is None
                       else {**(winner_ov or {}), "max_ramp": winner_val}),
            "winner_score": None,
            "measured": (None if stats is None
                         else {k: round(v, 3) for k, v in stats.items()}),
            "computed_max_ramp": computed,
            "used_fallback_settings": winner_ov,
            "search": "bubble",
            "trials": indices,
        }
        self._save()
        self.log(f"stage {stage.name}: winner "
                 f"{None if winner_val is None else {**(winner_ov or {}), 'max_ramp': winner_val}}")

    def _run_constraint_stage(self, stage: StageSpec) -> None:
        """Constraint-only sweep: first value with ZERO failures wins."""
        profile = self._stage_profile(stage)
        param = self.spec.parameters[stage.sweep]
        indices, winner_val = [], None
        for v in param.values:
            e = self._trial(TrialPlan(
                stage=stage.name, kind="trial",
                overrides=self._merged(stage, {param.name: v}),
                profile=profile))
            indices.append(e["index"])
            if not e.get("disqualified"):
                winner_val = v
                break
        if winner_val is not None:
            self.manifest["incumbent"] = self._merged(
                stage, {param.name: winner_val})
        self.manifest["stages"][stage.name] = {
            "winner": (None if winner_val is None
                       else {param.name: winner_val}),
            "winner_score": None,
            "trials": indices,
        }
        self._save()
        self.log(f"stage {stage.name}: winner "
                 f"{None if winner_val is None else {param.name: winner_val}}")

    # -- finals -------------------------------------------------------------
    def _run_abba_block(self, winner_ov: dict, profile: Profile,
                        rep: int) -> tuple[list[dict], list[float], int]:
        """One ABBA block -> (block entries, paired deltas, winner_fail_count)."""
        pattern = [("final_winner", winner_ov), ("final_default", {}),
                   ("final_default", {}), ("final_winner", winner_ov)]
        while True:     # a pack swap mid-block restarts the whole block
            swaps_before = self._swap_count()
            block: list[dict] = []
            for kind, ov in pattern:
                e = self._trial(TrialPlan(stage="finals", kind=kind,
                                          overrides=ov, profile=profile,
                                          repeat=rep))
                block.append(e)
                if self._swap_count() != swaps_before and len(block) > 1:
                    break
            if self._swap_count() == swaps_before or len(block) == 1:
                break
            for e in block:
                e["discarded"] = True
            self._save()
            self.log(f"finals block {rep}: pack swap mid-block - "
                     "restarting the block")
        w = [e for e in block if e["kind"] == "final_winner"]
        d = [e for e in block if e["kind"] == "final_default"]
        fails = sum(1 for e in w if e.get("disqualified"))
        deltas: list[float] = []
        for we, de in zip(w, d):
            if (we.get("score_raw") is not None
                    and de.get("score_raw") is not None):
                deltas.append(we["score_raw"] - de["score_raw"])
        return block, deltas, fails

    def _finals_delta_threshold(self, default_scores: list[float]) -> float:
        """Min median paired Δ (g/W) required to confirm the winner."""
        pct = self.spec.finals.min_delta_pct
        if not default_scores or pct <= 0:
            return 0.0
        ref = statistics.median(default_scores)
        if ref is None or ref <= 0:
            return 0.0
        return ref * pct / 100.0

    def _run_finals(self) -> dict:
        finals = self.spec.finals
        profile = self._profile_by_name(finals.profile)
        winner_ov = self.incumbent
        deltas: list[float] = []
        default_scores: list[float] = []
        winner_fails = 0
        indices: list[int] = []
        blocks_run = 0

        def run_blocks(n: int, rep_offset: int) -> None:
            nonlocal winner_fails, blocks_run
            for i in range(n):
                rep = rep_offset + i
                block, dlt, fails = self._run_abba_block(
                    winner_ov, profile, rep)
                indices.extend(e["index"] for e in block)
                deltas.extend(dlt)
                winner_fails += fails
                for e in block:
                    if (e["kind"] == "final_default"
                            and e.get("score_raw") is not None
                            and not e.get("disqualified")):
                        default_scores.append(e["score_raw"])
                blocks_run += 1

        run_blocks(finals.repeats, 0)
        median_delta = statistics.median(deltas) if deltas else None
        threshold = self._finals_delta_threshold(default_scores)

        # Close call: positive but below 2× threshold -> extra ABBA blocks.
        if (finals.extra_repeats_if_close > 0
                and median_delta is not None
                and median_delta > 0
                and median_delta <= max(threshold * 2.0, 1e-9)):
            self.log(
                f"finals: median Δ {median_delta:.4f} g/W is close to the "
                f"min-delta threshold {threshold:.4f}; running "
                f"{finals.extra_repeats_if_close} extra ABBA block(s)")
            run_blocks(finals.extra_repeats_if_close, blocks_run)
            median_delta = statistics.median(deltas) if deltas else None
            threshold = self._finals_delta_threshold(default_scores)

        startup_ok = True
        st = None
        if finals.startup_check:
            sp = startup_profile(self.spec)
            e = self._trial(TrialPlan(stage="finals", kind="final_startup",
                                      overrides=winner_ov, profile=sp))
            indices.append(e["index"])
            st = e.get("startup")
            startup_ok = not e.get("disqualified")

        # Optional high-throttle constraint hold through a known desync band
        # (e.g. t70 on this bench). Failing this unconfirms the winner so we
        # never ship an efficiency peak that cannot hold in that band.
        high_throttle_info = None
        high_throttle_ok = True
        if finals.high_throttle is not None and winner_ov:
            ht_prof = high_throttle_profile(
                self.spec, finals.high_throttle,
                finals.high_throttle_dwell_s)
            e = self._trial(TrialPlan(
                stage="finals", kind="final_high_throttle",
                overrides=winner_ov, profile=ht_prof))
            indices.append(e["index"])
            high_throttle_ok = not e.get("disqualified")
            high_throttle_info = {
                "throttle": finals.high_throttle,
                "dwell_s": finals.high_throttle_dwell_s,
                "ok": high_throttle_ok,
                "disqualified": e.get("disqualified"),
                "trial_index": e["index"],
            }
            if not high_throttle_ok:
                self.log(f"finals: high-throttle @{finals.high_throttle} "
                         f"FAILED ({e.get('disqualified')}) - will not "
                         "confirm winner")

        # Confirm only with a Δ clearly above noise (min_delta_pct of the
        # default-leg score). threshold=0 still requires a strictly positive Δ.
        delta_ok = (median_delta is not None and median_delta > threshold
                    and median_delta > 0)
        confirmed = (bool(winner_ov) and winner_fails == 0 and startup_ok
                     and high_throttle_ok and delta_ok)
        if (median_delta is not None and median_delta > 0 and not delta_ok
                and winner_fails == 0 and startup_ok and high_throttle_ok):
            self.log(
                f"finals: median Δ {median_delta:.4f} g/W below min-delta "
                f"threshold {threshold:.4f} g/W "
                f"({finals.min_delta_pct}% of default score) - not confirmed")
        result = {
            "winner_overrides": winner_ov,
            "median_paired_delta": (None if median_delta is None
                                    else round(median_delta, 5)),
            "paired_deltas": [round(x, 5) for x in deltas],
            "min_delta_threshold": round(threshold, 5),
            "min_delta_pct": finals.min_delta_pct,
            "abba_blocks": blocks_run,
            "winner_constraint_failures": winner_fails,
            "startup": st,
            "high_throttle": high_throttle_info,
            "confirmed": confirmed,
            "trials": indices,
        }
        self.manifest["result"] = result
        self._save()
        return result

    # -- top level -----------------------------------------------------------
    def run(self) -> dict:
        spec = self.spec
        self.log(f"tune session {spec.name!r} -> {self.out} "
                 f"({self.backend.mode}, eeprom @ "
                 f"0x{self.backend.eeprom_address:08x})")
        # The incumbent is REPLAYED, not loaded: the manifest holds its
        # latest value, but deterministic planning must see it evolve stage
        # by stage exactly as the original session did.
        self.manifest["incumbent"] = {}
        e = self._trial(TrialPlan(stage="baseline", kind="baseline",
                                  overrides={}, profile=probe_profile(spec)))
        if e.get("disqualified"):
            self.log(f"WARNING: baseline run disqualified: "
                     f"{e['disqualified']}; retrying once")
            e = self._trial(TrialPlan(stage="baseline", kind="baseline",
                                      overrides={},
                                      profile=probe_profile(spec), repeat=1))
        if e.get("disqualified"):
            self._quarantine_from(0)
            raise TunePaused(
                "baseline (default settings) disqualified twice: "
                f"{e['disqualified']} - fix the rig or spec limits, then "
                f"resume with 'hwci tune --resume {self.out}'")
        if self.manifest["jitter_reference"] is None:
            self.manifest["jitter_reference"] = e.get("jitter_pct")
            if e.get("jitter_pct") is None:
                self.log("WARNING: baseline has no zc-jitter data; the "
                         "jitter regression gate is OFF for this session")
            self._save()

        for stage in spec.stages:
            if stage.measure:
                self._run_measure_stage(stage)
            elif stage.constraint_only:
                self._run_constraint_stage(stage)
            else:
                self._run_scored_stage(stage)

        result = self._run_finals()
        best = (self.base.apply(result["winner_overrides"], self._offsets)
                if result["confirmed"] else self.base)
        srows = tunereport.settings_rows(
            self.base, best, self.spec.parameters, self._offsets)
        # Leave the DEVICE on the session's verdict too.
        self.backend.program(best.to_bytes(), self.out / "best_settings.bin")
        (self.out / "settings_diff.md").write_text(
            tunereport.diff_md(self.base, best))
        (self.out / "report.md").write_text(
            tunereport.report_md(self.manifest, result, srows, self.out))
        card_md, card_json = tunereport.write_pilot_card(
            self.out, self.manifest, result, srows, self.base, best)
        self.log(f"pilot card: {card_md}")
        pdf = reportmod.write_tune_pdf(
            self.out, self.manifest, result, self.base.diff(best),
            settings_rows=srows, log=self.log)
        if pdf is not None:
            self.log(f"PDF report: {pdf}")
        self.log(f"winner {result['winner_overrides'] or '{}'} "
                 f"{'CONFIRMED' if result['confirmed'] else 'NOT confirmed'} "
                 f"(median paired delta: {result['median_paired_delta']}, "
                 f"threshold: {result.get('min_delta_threshold')})")
        return result

    # -- reporting helpers kept for tests that poke private methods ----------
    def _diff_md(self, best: Settings) -> str:
        return tunereport.diff_md(self.base, best)

    def _settings_rows(self, best: Settings) -> list[dict]:
        return tunereport.settings_rows(
            self.base, best, self.spec.parameters, self._offsets)

    def _report_md(self, result: dict, settings_rows_list: list[dict]) -> str:
        return tunereport.report_md(
            self.manifest, result, settings_rows_list, self.out)
