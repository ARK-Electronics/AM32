"""Objective scoring and hard-constraint checks for tune trials."""
from __future__ import annotations

import numpy as np

from .. import metrics as metricsmod
from ..config import Profile
from ..model import RunResult
from .spec import ConstraintsSpec, ObjectiveSpec


def objective_score(m: dict, objective: ObjectiveSpec) -> float | None:
    """Weighted mean of eff_gf_per_w over steady points at meaningful power.

    Points below ``min_power_w`` are excluded entirely - on the bench they
    are the ratio of two small noisy numbers (a 2.3 W point swung -73 %
    run-to-run on unchanged firmware, see hwci/baseline.py). Labels missing
    from ``weights`` get weight 1.0. None if no point qualifies.
    """
    num = den = 0.0
    for p in m.get("steady_points", []):
        eff, pw = p.get("eff_gf_per_w"), p.get("elec_power_w")
        if eff is None or pw is None or eff != eff or pw != pw:
            continue
        if pw < objective.min_power_w:
            continue
        w = float(objective.weights.get(p["segment"], 1.0))
        num += w * eff
        den += w
    return num / den if den > 0 else None


def check_constraints(m: dict, meta: dict, cons: ConstraintsSpec, *,
                      jitter_reference: float | None,
                      settings_verified: bool,
                      startup: dict | None = None,
                      min_start_rpm: float | None = None) -> list[str]:
    """Hard-constraint check -> list of failure strings (empty = pass).

    A failing trial is DISQUALIFIED, never scored: a desyncing run can post a
    great g/W (undriven windmilling draws no power), so scoring it at all
    would reward exactly the behaviour the tuner must avoid.
    """
    fails: list[str] = []
    if meta.get("aborted"):
        fails.append(f"run aborted: {meta['aborted']}")
    if not settings_verified:
        fails.append("settings readback mismatch (page on device != intended)")
    d = m.get("demag", {})
    if d.get("event_count", 0) > cons.max_demag_events:
        fails.append(f"demag events {d.get('event_count')} > "
                     f"{cons.max_demag_events}")
    if d.get("bemf_timeout_samples", 0) > cons.max_bemf_timeout_samples:
        fails.append(f"bemf timeout samples {d.get('bemf_timeout_samples')} > "
                     f"{cons.max_bemf_timeout_samples}")
    s = m.get("summary", {})
    for key, bound in (("max_fet_temp_c", cons.max_fet_temp_c),
                       ("max_motor_temp_c", cons.max_motor_temp_c)):
        v = s.get(key)
        if bound is not None and v is not None and v == v and v > bound:
            fails.append(f"{key} {v:.1f} > {bound:.1f}")
    j = s.get("worst_zc_jitter_pct")
    if (jitter_reference is not None and j is not None
            and j > jitter_reference
            * (1.0 + cons.jitter_max_regression_pct / 100.0)):
        fails.append(
            f"zc jitter {j:.2f}% regressed > {cons.jitter_max_regression_pct}% "
            f"vs default-settings anchor ({jitter_reference:.2f}%)")
    if startup is not None:
        if startup["failed"] > cons.startup.max_failed:
            fails.append(f"failed starts {startup['failed']}/"
                         f"{startup['cycles']} > {cons.startup.max_failed}")
    elif min_start_rpm is not None:
        pts = m.get("steady_points", [])
        rpms = [p.get("rpm") for p in pts
                if p.get("rpm") is not None and p.get("rpm") == p.get("rpm")]
        if pts and (not rpms or max(rpms) < min_start_rpm):
            fails.append("failed start: no steady point reached "
                         f"{min_start_rpm:.0f} rpm")
    return fails


def startup_stats(result: RunResult, profile: Profile,
                  min_rpm: float = 1000.0) -> dict:
    """Count failed starts in a startup_profile run.

    A cycle failed if by the END of its spin segment neither the stand RPM
    nor the perf eRPM/pole_pairs exceeded ``min_rpm``.

    TODO: replace with PR #22's startup_reliability metric when merged.
    """
    rows = result.rows
    seg = np.array([r.get("segment") for r in rows], dtype=object)
    stand_rpm = metricsmod._col(rows, "stand_rpm")
    e_rpm = metricsmod._col(rows, "perf_e_rpm")
    pp = int(result.meta.get("pole_pairs") or profile.pole_pairs)
    cycles = 0
    failed: list[str] = []
    for s in profile.segments:
        if not s.label.startswith("spin"):
            continue
        cycles += 1
        idx = np.where(seg == s.label)[0]
        if idx.size == 0:
            failed.append(s.label)
            continue
        tail = idx[-max(1, idx.size // 4):]     # end of the spin segment
        vals = np.concatenate([stand_rpm[tail],
                               e_rpm[tail] / pp])   # column is true eRPM
        vals = vals[~np.isnan(vals)]
        if not (vals.size and float(vals.max()) >= min_rpm):
            failed.append(s.label)
    return {"cycles": cycles, "failed": len(failed),
            "failed_segments": failed, "min_rpm": min_rpm}
