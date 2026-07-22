"""Measure plant sustain duty and map it to ``minimum_duty_cycle``.

EEPROM ``minimum_duty_cycle`` (offset 6) is the PWM duty *floor* applied at
any non-zero throttle. Firmware (Src/settings.c) multiplies the eeprom unit
by 10 to get duty counts out of 2000; the throttle map is then
(Src/control_loop.c, non-sine)::

    duty = min_duty                 if input <= 47
    duty = min_duty + (input-47)*slope   otherwise
    slope = (2000 - min_duty) / (2047 - 47)

So at DShot 48 (first real step) duty ≈ floor alone. The auto-tuner must
choose a floor high enough that **DShot idle (48)** sustains the prop — not
merely that some mid-low host-percent hold spun during a crawl.

Host throttle on the Flight Stand maps as::

    DShot = 48 + throttle * (2047 - 48)   for throttle > 0
    DShot = 0                             for throttle == 0  (disarm)

so host 2% ≈ DShot 88, which is *above* the kick-loop band (≈48–65) the
pilot cares about. Measure/verify profiles therefore speak in DShot units.
"""
from __future__ import annotations

import math
import statistics
from typing import Optional

# DShot / duty map constants matching AM32 + the Flight Stand rig mapping.
DSHOT_IDLE = 48
DSHOT_MAX = 2047
DUTY_MAX = 2000
# Host throttle just above zero maps near DShot 48; thr==0 is disarm (DShot 0).
_HOST_EPS = 1.0 / (DSHOT_MAX - DSHOT_IDLE)   # → raw ≈ 49 if used as thr


def dshot_to_host_throttle(dshot: float) -> float:
    """Host throttle fraction that commands approximately ``dshot``.

    ``throttle == 0`` is reserved for disarm (DShot 0). The first real step
    (DShot 48) is reached with a tiny positive fraction; larger values use
    the linear esc_min..esc_max map.
    """
    d = float(dshot)
    if d <= 0:
        return 0.0
    if d <= DSHOT_IDLE:
        # Smallest positive step: Flight Stand raw = 48 + thr*1999.
        # thr = 0.25/1999 keeps raw in (48, 49) so the ESC sees idle-band.
        return 0.25 / (DSHOT_MAX - DSHOT_IDLE)
    return (d - DSHOT_IDLE) / (DSHOT_MAX - DSHOT_IDLE)


def host_throttle_to_dshot(throttle: float) -> float:
    """Inverse of :func:`dshot_to_host_throttle` (0 → 0, else ≥ 48)."""
    thr = max(0.0, min(1.0, float(throttle)))
    if thr <= 0.0:
        return 0.0
    return DSHOT_IDLE + thr * (DSHOT_MAX - DSHOT_IDLE)


def duty_counts_at_dshot(dshot: float, eeprom_s: int) -> float:
    """Firmware duty counts (0..2000) for a DShot input and eeprom min-duty."""
    min_duty = max(0, int(eeprom_s)) * 10
    d = float(dshot)
    if d <= 47:
        return float(min_duty)
    slope = (DUTY_MAX - min_duty) / (DSHOT_MAX - 47)
    return min_duty + (d - 47) * slope


def compute_min_duty_for_idle(sustain_dshot: float, *, lo: int, hi: int,
                              margin: float = 1.15,
                              measure_eeprom: int = 1) -> int:
    """EEPROM ``minimum_duty_cycle`` so DShot idle duty covers the plant need.

    ``sustain_dshot`` is the lowest DShot that still spun during a measure
    run with ``measure_eeprom`` programmed (typically the floor, 1). The
    duty at that point is the plant's sustain requirement; we raise the
    floor until duty at :data:`DSHOT_IDLE` meets that need times ``margin``.
    """
    if sustain_dshot <= 0:
        return lo
    d_need = duty_counts_at_dshot(sustain_dshot, measure_eeprom)
    target = d_need * max(margin, 0.1)
    # duty(DSHOT_IDLE, S) = S*10 + (48-47)*slope ≈ S*10 + (2000-S*10)/2000
    # Solve S*10 + (2000 - S*10)/2000 >= target  →  S*10 * 1999/2000 + 1 >= target
    # → S*10 >= (target - 1) * 2000/1999
    raw_counts = max(0.0, (target - 1.0) * DUTY_MAX / (DSHOT_MAX - 47))
    setting = int(math.ceil(raw_counts / 10.0 - 1e-9))
    # Also never go below ceil(target/10): the pure-floor approximation.
    setting = max(setting, int(math.ceil(target / 10.0 - 1e-9)))
    return max(lo, min(hi, setting))


# Back-compat alias used by older tests / call sites.
def compute_min_duty(sustain_throttle: float, *, lo: int, hi: int,
                     margin: float = 1.15) -> int:
    """Deprecated host-throttle form; prefer :func:`compute_min_duty_for_idle`.

    Treats host throttle as duty fraction (S/200) — wrong for DShot idle
    targeting, kept only so older unit tests and call sites keep working.
    """
    if sustain_throttle <= 0:
        return lo
    raw = sustain_throttle * 200.0 * max(margin, 0.1)
    setting = int(math.ceil(raw - 1e-9))
    return max(lo, min(hi, setting))


def _fget(r: dict, key: str) -> Optional[float]:
    v = r.get(key)
    try:
        return float(v) if v not in (None, "") else None
    except (TypeError, ValueError):
        return None


def _hold_is_driven(stand_rpm: float, e_rpm: Optional[float],
                    current_a: float, *,
                    min_rpm: float, pole_pairs: int,
                    min_current_a: float) -> bool:
    """True when the ESC is electrically tracking a spinning rotor.

    Rejects:
    * pure coast (stand RPM, ~0 current, eRPM collapsed)
    * kick / false-lock (running=1 but eRPM stuck far from stand RPM)
    Idle drive on a big prop may only draw ~0.03–0.05 A, so the current
    floor stays low; eRPM↔stand agreement is the main gate.
    """
    if stand_rpm < min_rpm:
        return False
    if current_a < min_current_a:
        return False
    if e_rpm is None or pole_pairs <= 0:
        return True
    mech_e = e_rpm / pole_pairs
    # Electrical path must also be above the floor (not a phantom lock at
    # a few hundred eRPM while the prop freewheels / kick-loops).
    if mech_e < min_rpm * 0.75:
        return False
    err = abs(stand_rpm - mech_e)
    # Kick signature on this bench: stand ~350-450 RPM with eRPM stuck
    # near 2.0-2.5k (mech ~300) — relative error 15-30%. True closed
    # loop tracks within a few percent (see d120: 601 vs 600).
    if err > max(80.0, 0.15 * stand_rpm):
        return False
    return True


def sustain_dshot_from_rows(
        rows: list[dict], *,
        min_rpm: float = 400.0,
        pole_pairs: int = 7,
        min_current_a: float = 0.02) -> Optional[dict]:
    """Lowest steady-segment DShot that *drives* sustained rotation.

    Segments come from ``throttle_cmd`` (→ DShot via the Flight Stand map).
    Prefer a **descending** crawl (spin up, step down): that tests sustain
    once spinning. An ascending-from-coast ladder tests *startup* at each
    step and needs far more duty (startup_power boost path).
    """
    segs: dict[str, list[dict]] = {}
    order: list[str] = []
    for r in rows:
        lbl = str(r.get("segment") or "")
        if lbl not in segs:
            segs[lbl] = []
            order.append(lbl)
        segs[lbl].append(r)

    # (dshot, throttle, label, med_rpm, med_current, driven)
    holds: list[tuple[float, float, str, float, float, bool]] = []
    for lbl in order:
        seg = segs[lbl]
        if not seg:
            continue
        thr = _fget(seg[-1], "throttle_cmd")
        if thr is None or thr <= 0.0:
            continue
        if lbl in ("idle", "spinup", "spool", "coast", "rampdn") \
                or lbl.startswith("r_"):
            continue
        dshot = host_throttle_to_dshot(thr)
        tail = seg[len(seg) // 2:]
        stands: list[float] = []
        e_rpms: list[float] = []
        currents: list[float] = []
        for r in tail:
            stand = _fget(r, "stand_rpm")
            e_rpm = _fget(r, "perf_e_rpm")
            cur = _fget(r, "stand_current_a")
            if stand is not None:
                stands.append(stand)
            if e_rpm is not None:
                e_rpms.append(e_rpm)
            if cur is not None:
                currents.append(abs(cur))
        if not stands:
            continue
        med_stand = statistics.median(stands)
        med_e = statistics.median(e_rpms) if e_rpms else None
        med_i = statistics.median(currents) if currents else 0.0
        driven = _hold_is_driven(
            med_stand, med_e, med_i,
            min_rpm=min_rpm, pole_pairs=pole_pairs,
            min_current_a=min_current_a)
        holds.append((dshot, thr, lbl, med_stand, med_i, driven))

    if not holds:
        return None

    sustained = [h for h in holds if h[5]]
    if not sustained:
        return None
    dshot, thr, lbl, rpm, med_i, _ = min(sustained, key=lambda x: x[0])
    failed = [h for h in holds if not h[5]]
    return {
        "sustain_dshot": dshot,
        "sustain_throttle": thr,
        "sustain_segment": lbl,
        "sustain_rpm": rpm,
        "sustain_current_a": med_i,
        "sustain_duty_counts": duty_counts_at_dshot(dshot, 1),
        "holds": len(holds),
        "failed_holds": len(failed),
        "lowest_dshot": min(d for d, *_ in holds),
        "highest_dshot": max(d for d, *_ in holds),
    }


# Back-compat name used by session.py / tests before the DShot rename.
def sustain_throttle_from_rows(
        rows: list[dict], *,
        min_rpm: float = 400.0,
        pole_pairs: int = 7) -> Optional[dict]:
    """Alias of :func:`sustain_dshot_from_rows` (same return dict)."""
    return sustain_dshot_from_rows(
        rows, min_rpm=min_rpm, pole_pairs=pole_pairs)
