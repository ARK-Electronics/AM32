"""Inline profiles used by the auto-tuner (probe, startup, step, ramp)."""
from __future__ import annotations

import yaml

from ..config import PROFILES_DIR, Profile, profile_from_dict
from .spec import TuneSpec

# Transient current headroom for the ramp stage's snap profiles (step_profile,
# ramp_measure_profile): a legitimate 0.3->0.95 (or 0.2->0.55) snap draws well
# above the STEADY probe.safety.max_current_a, so these profiles floor their
# own limit at this value regardless of the spec's steady-state setting.
# Raised 55 -> 100 (2026-07-07 bench session, JS 2306 1800KV + 5" prop on 6S).
RAMP_TRANSIENT_MAX_CURRENT_A = 100.0


def probe_profile(spec: TuneSpec) -> Profile:
    """The tuner's inner probe (~28 s): tune_probe.yaml with the spec's
    points/dwell/safety overrides applied."""
    d = yaml.safe_load((PROFILES_DIR / "tune_probe.yaml").read_text())
    if spec.probe.points:
        # Keep the low-throttle warmup staircase (see the profile YAML for
        # why it must exist) and rebuild only the measured points.
        segs = [
            {"label": "idle", "throttle": 0.0, "duration_s": 2.0},
            {"label": "w10", "throttle": 0.10, "duration_s": 1.5, "ramp": True},
            {"label": "w20", "throttle": 0.20, "duration_s": 1.5},
        ]
        prev = 0.20
        for label, thr in spec.probe.points.items():
            thr = float(thr)
            # Steps > 10% throttle get a ramp segment: a 0.5->0.7 snap drew a
            # 43-75 A one-sample transient on the 6S bench (harness abort at
            # 40 A), while efficiency_sweep's unramped 10% staircase is
            # bench-proven safe. The dwell stays constant-throttle so the
            # steady tail is unaffected.
            if abs(thr - prev) > 0.10 + 1e-9:
                segs.append({"label": f"r_{label}", "throttle": thr,
                             "duration_s": 1.0, "ramp": True})
            segs.append({"label": label, "throttle": thr,
                         "duration_s": spec.probe.dwell_s, "steady": True})
            prev = thr
        segs.append({"label": "rampdn", "throttle": 0.0, "duration_s": 2.0,
                     "ramp": True})
        d["segments"] = segs
    else:
        for seg in d["segments"]:
            if seg.get("steady"):
                seg["duration_s"] = spec.probe.dwell_s
    if spec.probe.safety:
        d["safety"] = {**(d.get("safety") or {}), **spec.probe.safety}
    return profile_from_dict(d)


def startup_profile(spec: TuneSpec) -> Profile:
    """Inline minimal startup-reliability profile: N cycles of
    {spin low for 1.5 s -> stop 2.5 s}.

    TODO: PR #22's startup_reliability profile/metric is not on this branch;
    delegate to it (profile + its richer per-cycle metric) once merged.
    """
    st = spec.constraints.startup
    segs = [{"label": "idle", "throttle": 0.0, "duration_s": 1.0}]
    for i in range(st.cycles):
        segs.append({"label": f"spin{i}", "throttle": st.spin_throttle,
                     "duration_s": 1.5})
        segs.append({"label": f"stop{i}", "throttle": 0.0, "duration_s": 2.5})
    return profile_from_dict({
        "name": "tune_startup",
        "description": "inline startup-reliability check (auto-tuner)",
        "sample_rate_hz": 100.0,
        "segments": segs,
        "safety": spec.probe.safety or None,
    })


def step_profile(spec: TuneSpec) -> Profile:
    """Step-stress profile for the max_ramp constraint stage: aggressive
    snaps into a high-current regime FROM A SPINNING STATE (0.30 hold).

    Snapping from a stop (as demag_step_stress does) makes the host demag
    detector read the spool-up's long commutation intervals as spike events
    even when nothing desynced; stepping from 0.30 keeps the detector's
    median-interval reference honest, so only a real loss of sync (bemf
    timeouts, interval blow-up, RPM collapse) disqualifies a max_ramp value.
    """
    return profile_from_dict({
        "name": "tune_step",
        "description": "inline step-stress for the max_ramp stage (auto-tuner)",
        "sample_rate_hz": 100.0,
        "segments": [
            {"label": "idle", "throttle": 0.0, "duration_s": 2.0},
            {"label": "spool", "throttle": 0.30, "duration_s": 2.0, "ramp": True},
            {"label": "hold30a", "throttle": 0.30, "duration_s": 2.0},
            {"label": "snap95a", "throttle": 0.95, "duration_s": 1.5},
            {"label": "hold30b", "throttle": 0.30, "duration_s": 2.0},
            {"label": "snap95b", "throttle": 0.95, "duration_s": 1.5},
            {"label": "hold30c", "throttle": 0.30, "duration_s": 2.0},
            {"label": "rampdn", "throttle": 0.0, "duration_s": 1.5, "ramp": True},
        ],
        # The snaps are the point of this profile, and a legitimate
        # 0.3->0.95 snap transient can draw into the 80-100 A range on this
        # bench (see RAMP_TRANSIENT_MAX_CURRENT_A). Probe-level current
        # limits would abort every candidate on that transient before demag
        # is even assessed, so give the current limit snap headroom; all
        # other probe safety limits apply unchanged.
        "safety": {**(spec.probe.safety or {}),
                   "max_current_a": max(
                       (spec.probe.safety or {}).get("max_current_a") or 0.0,
                       RAMP_TRANSIENT_MAX_CURRENT_A)},
    })


def ramp_measure_profile(spec: TuneSpec) -> Profile:
    """Measurement profile for the mech-ramp stage: two moderate snap steps
    from a spinning state (0.20 -> 0.55), sampled at 200 Hz.

    The trial runs with max_ramp at the field maximum so firmware duty slew
    is not the limiter: the eRPM rise time is then the POWERTRAIN's (rotor
    inertia + prop aero load), and the current overshoot measures how much
    a leading duty costs. Two reps -> median estimates. The step tops out
    at 0.55 to stay clear of high-RPM sync margins (this bench desyncs
    arriving at fresh-pack t70)."""
    safety = dict(spec.probe.safety or {})
    # deliberate snaps: same transient headroom rationale as step_profile
    safety["max_current_a"] = max(safety.get("max_current_a") or 0.0,
                                  RAMP_TRANSIENT_MAX_CURRENT_A)
    return profile_from_dict({
        "name": "tune_ramp_measure",
        "description": "inline powertrain step-response measurement "
                       "(auto-tuner mech-ramp stage)",
        "sample_rate_hz": 200.0,
        "segments": [
            {"label": "idle",     "throttle": 0.00, "duration_s": 2.0},
            {"label": "spool",    "throttle": 0.20, "duration_s": 2.0,
             "ramp": True},
            {"label": "hold_lo",  "throttle": 0.20, "duration_s": 1.5},
            {"label": "step_up",  "throttle": 0.55, "duration_s": 2.5},
            {"label": "drop",     "throttle": 0.20, "duration_s": 2.0},
            {"label": "step_up2", "throttle": 0.55, "duration_s": 2.5},
            {"label": "rampdn",   "throttle": 0.00, "duration_s": 1.5,
             "ramp": True},
        ],
        "safety": safety,
    })


def _dshot_hold(label: str, dshot: int, duration_s: float,
                *, steady: bool = True) -> dict:
    """Steady hold at an absolute DShot setpoint (via host throttle map)."""
    from .minduty import dshot_to_host_throttle
    seg = {"label": label, "throttle": dshot_to_host_throttle(dshot),
           "duration_s": duration_s}
    if steady:
        seg["steady"] = True
    return seg


def min_duty_measure_profile(spec: TuneSpec) -> Profile:
    """Descending DShot crawl for min-duty: spin up, step through the idle
    band (100 → 48), park.

    Programs ``minimum_duty_cycle`` at the field floor so the firmware floor
    cannot mask the plant; analysis finds the lowest DShot that still spins
    and maps that duty need onto a floor that sustains at DShot 48.
    """
    from .minduty import dshot_to_host_throttle
    safety = dict(spec.probe.safety or {})
    # Descending crawl: spin up first, then step down through the idle band.
    # That tests *sustain* once closed-loop (what min_duty is for). Climbing
    # from a stop tests *startup* at each DShot and needs the startup_power
    # boost path — a different (higher) duty requirement.
    dshots = (140, 120, 100, 80, 70, 65, 60, 55, 52, 50, 48)
    segs: list[dict] = [
        {"label": "idle", "throttle": 0.00, "duration_s": 2.0},
        {"label": "spinup", "throttle": dshot_to_host_throttle(220),
         "duration_s": 3.0, "ramp": True},
        _dshot_hold("hold_hi", 180, 2.0),
    ]
    for d in dshots:
        segs.append(_dshot_hold(f"d{d}", d, 3.0))
    segs.append({"label": "rampdn", "throttle": 0.00, "duration_s": 2.0,
                 "ramp": True})
    return profile_from_dict({
        "name": "tune_min_duty_measure",
        "description": "inline DShot-idle crawl for min-duty measurement "
                       "(auto-tuner)",
        "sample_rate_hz": 100.0,
        "segments": segs,
        "safety": safety or None,
    })


def min_duty_verify_profile(spec: TuneSpec) -> Profile:
    """Verify a programmed minimum_duty_cycle at DShot idle.

    Spin up, then hold DShot 55 / 50 / 48. With an adequate floor the
    firmware keeps duty above the plant sustain threshold even at DShot 48
    (first real throttle step); a too-low floor kick-loops here.
    """
    from .minduty import dshot_to_host_throttle
    safety = dict(spec.probe.safety or {})
    return profile_from_dict({
        "name": "tune_min_duty_verify",
        "description": "inline DShot-idle sustain check for min-duty "
                       "(auto-tuner)",
        "sample_rate_hz": 100.0,
        "segments": [
            {"label": "idle", "throttle": 0.00, "duration_s": 2.0},
            {"label": "spinup", "throttle": dshot_to_host_throttle(220),
             "duration_s": 3.0, "ramp": True},
            _dshot_hold("hold_hi", 120, 2.0),
            # Step down into idle — sustain, not cold-start at DShot 48.
            _dshot_hold("d55", 55, 3.5),
            _dshot_hold("d50", 50, 3.5),
            _dshot_hold("d48", 48, 4.0),
            {"label": "rampdn", "throttle": 0.00, "duration_s": 2.0,
             "ramp": True},
        ],
        "safety": safety or None,
    })


def high_throttle_profile(spec: TuneSpec, throttle: float,
                          dwell_s: float) -> Profile:
    """Constraint-only hold through a known desync band (e.g. t70).

    Warmup staircase then a single steady point; scored only via constraints
    (demag/bemf/abort/temps), never via the efficiency objective.
    """
    thr = float(throttle)
    dwell = float(dwell_s)
    safety = dict(spec.probe.safety or {})
    segs = [
        {"label": "idle", "throttle": 0.0, "duration_s": 2.0},
        {"label": "w10", "throttle": 0.10, "duration_s": 1.5, "ramp": True},
        {"label": "w20", "throttle": 0.20, "duration_s": 1.5},
    ]
    prev = 0.20
    # Climb in ~10% steps with ramps on larger jumps (same snap-safety as probe).
    for level in (0.30, 0.40, 0.50, 0.60):
        if thr <= level + 1e-9:
            break
        if abs(level - prev) > 0.10 + 1e-9:
            segs.append({"label": f"r_{int(level * 100)}", "throttle": level,
                         "duration_s": 1.0, "ramp": True})
        segs.append({"label": f"w{int(level * 100)}", "throttle": level,
                     "duration_s": 1.5})
        prev = level
    if abs(thr - prev) > 0.10 + 1e-9:
        segs.append({"label": "r_hold", "throttle": thr,
                     "duration_s": 1.0, "ramp": True})
    segs.append({"label": "hold", "throttle": thr, "duration_s": dwell,
                 "steady": True})
    segs.append({"label": "rampdn", "throttle": 0.0, "duration_s": 2.0,
                 "ramp": True})
    return profile_from_dict({
        "name": "tune_high_throttle",
        "description": "constraint-only high-throttle desync-band check",
        "sample_rate_hz": 100.0,
        "segments": segs,
        "safety": safety or None,
    })
