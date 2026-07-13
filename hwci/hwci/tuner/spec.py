"""Tune-spec YAML schema: strict validation and dataclasses."""
from __future__ import annotations

import dataclasses
from dataclasses import dataclass, field
from pathlib import Path

import yaml

from ..settings import resolve_field


class TuneSpecError(ValueError):
    """A tune spec failed strict validation (unknown key, bad value)."""


@dataclass
class PackSpec:
    min_resting_cell_v: float = 3.5
    prompt_on_swap: bool = True


@dataclass
class ThermalSpec:
    max_start_fet_temp_c: float | None = None
    cool_timeout_s: float = 600.0
    # When True (default), only call wait_for_cool if the previous trial's
    # FET temp was at/above the start limit - skips OpenOCD polling when the
    # ESC is already cool enough (large wall-clock win on short probes).
    cool_only_when_hot: bool = True


@dataclass
class ProbeSpec:
    points: dict[str, float] | None = None   # label -> throttle, ordered
    # Left at the efficiency_sweep-standard 6s: the 2026-07-07 noise-floor
    # characterization's much longer 15-20s holds gave tighter same-session
    # numbers in isolation, but the tuner's OWN noise floor (see
    # ObjectiveSpec.noise_floor_pct) is already down at 0.5-0.77% CV at 6s via
    # anchor-normalization + multi-point weighting - no observed need to pay
    # for a longer probe.
    dwell_s: float = 6.0
    safety: dict = field(default_factory=dict)


@dataclass
class ObjectiveSpec:
    weights: dict[str, float] = field(default_factory=lambda: {
        "t30": 1.0, "t50": 2.0, "t70": 1.0, "t90": 0.5})
    # Matches Thresholds.efficiency_min_power_w in hwci/baseline.py - the same
    # noise-floor characterization that sized that value (2026-07-07) confirms
    # 20W is still where the noise character changes on this bench.
    min_power_w: float = 20.0
    # Ties within this % of the best weighted score break on lower jitter,
    # then lower FET temp, then closest-to-default settings (see
    # pick_winner) - i.e. this is "how close counts as noise, not signal."
    # Empirically confirmed 2026-07-07 from real completed sessions:
    # anchor-to-anchor score CV was 0.50% / 0.77%; tightened to 2.0.
    noise_floor_pct: float = 2.0


@dataclass
class StartupSpec:
    cycles: int = 5
    max_failed: int = 0
    spin_throttle: float = 0.15
    min_rpm: float = 1000.0


@dataclass
class ConstraintsSpec:
    max_demag_events: int = 0
    max_bemf_timeout_samples: int = 0
    jitter_max_regression_pct: float = 25.0
    max_fet_temp_c: float | None = None
    max_motor_temp_c: float | None = None
    startup: StartupSpec = field(default_factory=StartupSpec)


@dataclass
class ParamSpec:
    name: str
    values: list[int]
    refine_step: int | None = None
    offset: int | None = None      # explicit byte offset (forward-compat)


@dataclass
class StageSpec:
    name: str
    sweep: str | None = None                    # parameter name to grid-sweep
    ab_candidates: list[dict] | None = None     # override dicts to A/B
    fixed: dict = field(default_factory=dict)   # forced during this stage only
    repeats: int = 1
    profile: str | None = None                  # None -> tune probe
    constraint_only: bool = False
    # "grid" tests every listed value; "climb" hill-climbs the sorted value
    # list from the incumbent (valid for unimodal responses like advance or
    # pwm frequency - typically halves the trial count of a wide grid).
    search: str = "grid"
    # "ramp_rate": measure the powertrain (mech time constant + transient
    # current per % of duty lead) with one instrumented step run, COMPUTE
    # max_ramp from the physics, then verify it on the step-stress profile.
    measure: str | None = None
    margin: float = 0.8       # fraction of the physics-derived rate to keep
    # Restrict a sweep to values within ±polish_radius (setting units) of
    # the current incumbent - used for a cheap post-modes advance re-climb.
    polish_radius: int | None = None


@dataclass
class FinalsSpec:
    profile: str = "efficiency_sweep"
    pattern: str = "ABBA"
    repeats: int = 2
    startup_check: bool = True
    # Optional constraint-only hold at this throttle (0..1) after the winner
    # is chosen - used to reject efficiency winners that desync in a known
    # bad band (e.g. 0.70 on the ARK 4IN1 + 5" bench). None disables.
    high_throttle: float | None = None
    high_throttle_dwell_s: float = 4.0
    # Minimum median paired Δ (winner - default) required to confirm, as a
    # percent of the median default-leg score in finals. 0 = any positive Δ.
    # Sized near the objective noise floor so lucky +0.02 g/W wins stay out.
    min_delta_pct: float = 1.0
    # If after ``repeats`` ABBA blocks the median Δ is positive but below
    # 2× the min-delta threshold (a close call), run this many extra blocks
    # before deciding.
    extra_repeats_if_close: int = 1


@dataclass
class TuneSpec:
    name: str
    description: str = ""
    battery_cells: int | None = None
    pack: PackSpec = field(default_factory=PackSpec)
    thermal: ThermalSpec = field(default_factory=ThermalSpec)
    probe: ProbeSpec = field(default_factory=ProbeSpec)
    objective: ObjectiveSpec = field(default_factory=ObjectiveSpec)
    constraints: ConstraintsSpec = field(default_factory=ConstraintsSpec)
    anchors_every: int = 5
    # When True, consecutive anchors that drift by more than
    # anchor_drift_pct temporarily halve anchors_every so pack sag is
    # tracked more tightly mid-stage.
    adaptive_anchors: bool = True
    anchor_drift_pct: float = 1.5
    parameters: dict[str, ParamSpec] = field(default_factory=dict)
    stages: list[StageSpec] = field(default_factory=list)
    finals: FinalsSpec = field(default_factory=FinalsSpec)

    def param_offsets(self) -> dict[str, int]:
        return {p.name: p.offset for p in self.parameters.values()
                if p.offset is not None}


def _strict(d: dict | None, ctx: str, known: dict) -> dict:
    """Return d after refusing unknown keys - a typo'd key must never
    silently become 'use the default' (mirrors load_rig strictness)."""
    d = d or {}
    if not isinstance(d, dict):
        raise TuneSpecError(f"tune spec: {ctx} must be a mapping, got {d!r}")
    unknown = sorted(set(d) - set(known))
    if unknown:
        raise TuneSpecError(
            f"tune spec: unknown key(s) {unknown} in {ctx}; "
            f"valid keys: {sorted(known)}")
    return d


def _build(cls, d: dict | None, ctx: str, nested: dict | None = None):
    known = {f.name: f for f in dataclasses.fields(cls)}
    d = dict(_strict(d, ctx, known))
    for key, sub in (nested or {}).items():
        if key in d:
            d[key] = sub(d[key])
    return cls(**d)


def _validate_overrides(ov: dict, params: dict[str, ParamSpec], ctx: str):
    if not isinstance(ov, dict):
        raise TuneSpecError(f"tune spec: {ctx}: overrides must be a mapping")
    for name, value in ov.items():
        offset = params[name].offset if name in params else None
        f = resolve_field(name, offset)
        if not f.lo <= int(value) <= f.hi:
            raise TuneSpecError(
                f"tune spec: {ctx}: {name}={value} outside firmware-valid "
                f"range [{f.lo}, {f.hi}]")


def tune_spec_from_dict(data: dict) -> TuneSpec:
    known = {f.name: f for f in dataclasses.fields(TuneSpec)}
    data = dict(_strict(data, "top level", known))
    if "name" not in data:
        raise TuneSpecError("tune spec: 'name' is required")

    data["pack"] = _build(PackSpec, data.get("pack"), "pack")
    data["thermal"] = _build(ThermalSpec, data.get("thermal"), "thermal")
    data["probe"] = _build(ProbeSpec, data.get("probe"), "probe")
    data["objective"] = _build(ObjectiveSpec, data.get("objective"),
                               "objective")
    data["constraints"] = _build(
        ConstraintsSpec, data.get("constraints"), "constraints",
        nested={"startup": lambda d: _build(StartupSpec, d,
                                            "constraints.startup")})
    data["finals"] = _build(FinalsSpec, data.get("finals"), "finals")
    ht = data["finals"].high_throttle
    if ht is not None and not (0.05 <= float(ht) <= 1.0):
        raise TuneSpecError(
            f"tune spec: finals.high_throttle {ht} outside [0.05, 1.0]")
    if data["finals"].high_throttle_dwell_s <= 0:
        raise TuneSpecError(
            "tune spec: finals.high_throttle_dwell_s must be > 0")
    if data["finals"].min_delta_pct < 0:
        raise TuneSpecError(
            "tune spec: finals.min_delta_pct must be >= 0")
    if data["finals"].extra_repeats_if_close < 0:
        raise TuneSpecError(
            "tune spec: finals.extra_repeats_if_close must be >= 0")
    if data.get("anchor_drift_pct") is not None and data["anchor_drift_pct"] <= 0:
        raise TuneSpecError("tune spec: anchor_drift_pct must be > 0")

    params: dict[str, ParamSpec] = {}
    for name, pd in (data.get("parameters") or {}).items():
        pd = _strict(pd, f"parameters.{name}",
                     {"values": None, "refine_step": None, "offset": None})
        if not pd.get("values"):
            raise TuneSpecError(f"tune spec: parameters.{name} needs 'values'")
        p = ParamSpec(name=name, values=[int(v) for v in pd["values"]],
                      refine_step=pd.get("refine_step"),
                      offset=pd.get("offset"))
        f = resolve_field(name, p.offset)   # raises on unknown/read-only
        for v in p.values:
            if not f.lo <= v <= f.hi:
                raise TuneSpecError(
                    f"tune spec: parameters.{name} value {v} outside "
                    f"firmware-valid range [{f.lo}, {f.hi}]")
        params[name] = p
    data["parameters"] = params

    stages: list[StageSpec] = []
    seen = set()
    for sd in data.get("stages") or []:
        s = _build(StageSpec, sd, "stages[]")
        if s.name in seen:
            raise TuneSpecError(f"tune spec: duplicate stage name {s.name!r}")
        seen.add(s.name)
        if sum(map(bool, (s.sweep, s.ab_candidates, s.measure))) != 1:
            raise TuneSpecError(
                f"tune spec: stage {s.name!r} needs exactly one of "
                "'sweep', 'ab_candidates' or 'measure'")
        if s.measure is not None and s.measure != "ramp_rate":
            raise TuneSpecError(
                f"tune spec: stage {s.name!r} measure {s.measure!r} is not "
                "'ramp_rate'")
        if s.measure and s.constraint_only:
            raise TuneSpecError(
                f"tune spec: stage {s.name!r}: 'measure' and "
                "'constraint_only' are mutually exclusive")
        if not 0.1 <= s.margin <= 2.0:
            raise TuneSpecError(
                f"tune spec: stage {s.name!r} margin {s.margin} outside "
                "[0.1, 2.0]")
        if s.search not in ("grid", "climb"):
            raise TuneSpecError(
                f"tune spec: stage {s.name!r} search {s.search!r} is not "
                "'grid' or 'climb'")
        if s.search == "climb" and (not s.sweep or s.constraint_only):
            raise TuneSpecError(
                f"tune spec: stage {s.name!r}: 'climb' only applies to "
                "scored sweep stages")
        if s.sweep and s.sweep not in params:
            raise TuneSpecError(
                f"tune spec: stage {s.name!r} sweeps unknown parameter "
                f"{s.sweep!r}; declare it under parameters:")
        if s.polish_radius is not None:
            if s.polish_radius < 0:
                raise TuneSpecError(
                    f"tune spec: stage {s.name!r} polish_radius must be >= 0")
            if not s.sweep or s.constraint_only:
                raise TuneSpecError(
                    f"tune spec: stage {s.name!r}: polish_radius only applies "
                    "to scored sweep stages")
        for ov in (s.ab_candidates or []) + [s.fixed]:
            _validate_overrides(ov, params, f"stage {s.name!r}")
        stages.append(s)
    data["stages"] = stages

    if data["finals"].pattern != "ABBA":
        raise TuneSpecError(
            f"tune spec: finals.pattern {data['finals'].pattern!r} is not "
            "supported (only 'ABBA')")
    return TuneSpec(**data)


def load_tune_spec(path: str | Path) -> TuneSpec:
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict):
        raise TuneSpecError(f"tune spec {path}: not a YAML mapping")
    try:
        return tune_spec_from_dict(data)
    except TuneSpecError as e:
        raise TuneSpecError(f"{path}: {e}") from e
