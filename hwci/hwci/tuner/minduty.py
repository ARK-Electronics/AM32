"""Measure the lowest sustainable throttle and map it to minimum_duty_cycle.

EEPROM ``minimum_duty_cycle`` (offset 6) is the PWM duty floor applied when
the host is above zero. Firmware (Src/settings.c) multiplies the eeprom unit
by 10 to get duty counts out of 2000, so one unit is 0.5% of full duty:

    duty_fraction ≈ S / 200

On a prop that needs ~2.9% duty to sustain (PR #41 crawl findings), the
default floor of 1-4 leaves DShot idle/low commands under-powered → endless
kick loop. The auto-tuner crawls down throttle, finds the lowest hold that
still spins, and programs ``ceil(t * 200 * margin)`` with margin ≥ 1 for
pack-sag headroom.
"""
from __future__ import annotations

import math
import statistics
from typing import Optional


# Host throttle fraction → continuous eeprom units (S/200 ≈ duty fraction).
THROTTLE_TO_EEPROM = 200.0


def _fget(r: dict, key: str) -> Optional[float]:
    v = r.get(key)
    try:
        return float(v) if v not in (None, "") else None
    except (TypeError, ValueError):
        return None


def sustain_throttle_from_rows(
        rows: list[dict], *,
        min_rpm: float = 400.0,
        pole_pairs: int = 7) -> Optional[dict]:
    """Lowest steady-segment host throttle that sustains rotation.

    Expects a descending crawl profile (spin-up, then steady holds). A hold
    sustains when the median of stand RPM and (perf eRPM / pole_pairs) in the
    second half of the segment is ≥ ``min_rpm``. Returns None when no hold
    sustains (or there are no steady segments).
    """
    segs: dict[str, list[dict]] = {}
    order: list[str] = []
    for r in rows:
        lbl = str(r.get("segment") or "")
        if lbl not in segs:
            segs[lbl] = []
            order.append(lbl)
        segs[lbl].append(r)

    holds: list[tuple[float, str, float]] = []  # (throttle, label, med_rpm)
    for lbl in order:
        seg = segs[lbl]
        if not seg:
            continue
        # Only steady-style holds: skip idle/spinup/rampdn labels that are
        # not constant-throttle crawl points. Prefer rows tagged steady via
        # a non-zero throttle_cmd held for the segment.
        thr = _fget(seg[-1], "throttle_cmd")
        if thr is None or thr < 0.015:
            continue
        if lbl in ("idle", "spinup", "spool", "rampdn") or lbl.startswith("r_"):
            continue
        tail = seg[len(seg) // 2:]
        rpms: list[float] = []
        for r in tail:
            stand = _fget(r, "stand_rpm")
            e_rpm = _fget(r, "perf_e_rpm")
            cands = []
            if stand is not None:
                cands.append(stand)
            if e_rpm is not None and pole_pairs > 0:
                cands.append(e_rpm / pole_pairs)
            if cands:
                rpms.append(max(cands))
        if not rpms:
            continue
        med = statistics.median(rpms)
        holds.append((thr, lbl, med))

    if not holds:
        return None

    # Prefer lower throttle when several holds pass; if none pass, None.
    sustained = [(t, lbl, rpm) for t, lbl, rpm in holds if rpm >= min_rpm]
    if not sustained:
        return None
    thr, lbl, rpm = min(sustained, key=lambda x: x[0])
    failed = [(t, lbl, rpm) for t, lbl, rpm in holds if rpm < min_rpm]
    return {
        "sustain_throttle": thr,
        "sustain_segment": lbl,
        "sustain_rpm": rpm,
        "holds": len(holds),
        "failed_holds": len(failed),
        "lowest_hold": min(t for t, _, _ in holds),
        "highest_hold": max(t for t, _, _ in holds),
    }


def compute_min_duty(sustain_throttle: float, *, lo: int, hi: int,
                     margin: float = 1.15) -> int:
    """EEPROM ``minimum_duty_cycle`` from a measured sustain throttle.

    ``margin`` ≥ 1.0 adds pack-sag / measurement headroom (1.15 ≈ +15%).
    Clamped to the firmware-valid range.
    """
    if sustain_throttle <= 0:
        return lo
    raw = sustain_throttle * THROTTLE_TO_EEPROM * max(margin, 0.1)
    setting = int(math.ceil(raw - 1e-9))
    return max(lo, min(hi, setting))
