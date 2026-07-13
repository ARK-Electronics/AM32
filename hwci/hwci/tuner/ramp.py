"""Physics-based max_ramp measurement from powertrain step response."""
from __future__ import annotations

import statistics
from typing import Optional


def mech_ramp_stats(rows: list[dict]) -> Optional[dict]:
    """Powertrain step-response estimates from a ``tune_ramp_measure`` run.

    Per step: first-order time constant ``tau_ms`` (63.2% eRPM crossing),
    mechanical slew (step height / tau) and the transient-current slope
    ``k_a_per_pct`` (peak current overshoot per % of commanded duty step -
    an upper bound on A per % of duty-lead, since the lead at the current
    peak is at most the full step). Median across steps; None if no usable
    step was found.
    """
    def fget(r, key):
        v = r.get(key)
        try:
            return float(v) if v not in (None, "") else None
        except (TypeError, ValueError):
            return None

    segs: dict[str, list[dict]] = {}
    for r in rows:
        segs.setdefault(str(r.get("segment")), []).append(r)

    ests = []
    for lbl, prev_lbl in (("step_up", "hold_lo"), ("step_up2", "drop")):
        step, prev = segs.get(lbl), segs.get(prev_lbl)
        if not step or not prev:
            continue
        tail = prev[-max(1, len(prev) // 5):]
        base_rpm = statistics.median(
            [v for r in tail if (v := fget(r, "perf_e_rpm")) is not None]
            or [float("nan")])
        base_i = statistics.median(
            [v for r in tail if (v := fget(r, "stand_current_a")) is not None]
            or [0.0])
        plat = step[-max(1, len(step) // 3):]
        hi_rpm = statistics.median(
            [v for r in plat if (v := fget(r, "perf_e_rpm")) is not None]
            or [float("nan")])
        hi_i = statistics.median(
            [v for r in plat if (v := fget(r, "stand_current_a")) is not None]
            or [0.0])
        if not (base_rpm == base_rpm and hi_rpm == hi_rpm):   # NaN guard
            continue
        rise = hi_rpm - base_rpm
        if rise < 1000.0:            # no real step -> nothing to measure
            continue
        t0 = fget(step[0], "t")
        thr0 = fget(prev[-1], "throttle_cmd") or 0.0
        thr1 = fget(step[-1], "throttle_cmd") or 0.0
        step_pct = abs(thr1 - thr0) * 100.0
        if t0 is None or step_pct < 5.0:
            continue
        target = base_rpm + 0.632 * rise
        t63 = next((t for r in step
                    if (v := fget(r, "perf_e_rpm")) is not None
                    and v >= target and (t := fget(r, "t")) is not None), None)
        if t63 is None:
            continue
        tau_ms = max((t63 - t0) * 1000.0, 5.0)   # floor at sampling grain
        i_pk = max([v for r in step
                    if (t := fget(r, "t")) is not None and t <= t0 + 0.5
                    and (v := fget(r, "stand_current_a")) is not None]
                   or [base_i])
        ests.append({
            "tau_ms": tau_ms,
            "slew_erpm_per_s": rise / (tau_ms / 1000.0),
            "k_a_per_pct": max(i_pk - base_i, 0.1) / step_pct,
            "i_peak_a": i_pk,
            "i_hi_a": hi_i,
            "rpm_lo": base_rpm,
            "rpm_hi": hi_rpm,
        })
    if not ests:
        return None
    return {k: statistics.median([e[k] for e in ests]) for k in ests[0]}


def compute_max_ramp(stats: dict, *, current_budget_a: float,
                     lo: int, hi: int, margin: float = 0.8) -> int:
    """max_ramp (0.1 %/ms units) from step-response physics.

    During a duty ramp at rate r the duty leads the mechanical state by
    ~r*tau once quasi-steady, and the transient current is ~k * lead. The
    fastest ramp whose worst-case transient stays inside the budget is
    r = budget / (k * tau); margin keeps a fraction of it.
    """
    lead_pct = current_budget_a / max(stats["k_a_per_pct"], 1e-3)
    rate_pct_per_ms = lead_pct / max(stats["tau_ms"], 1.0)
    setting = int(round(rate_pct_per_ms * 10.0 * margin))
    return max(lo, min(hi, setting))
