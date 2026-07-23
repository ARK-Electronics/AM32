"""Runner-level (host-side) safety enforcement - must trip on ANY rig, even
when the stand backend does no checking of its own."""
from hwci.config import Profile, Segment
from hwci.esc_telem.kiss import KissFrame
from hwci.flightstand.base import SafetyLimits
from hwci.flightstand.simulator import SimulatedStand
from hwci.runner import Sources, run_profile
from hwci.sim import MotorParams, RigSimulator
from hwci.throttle.flightstand_src import FlightStandThrottle


def _profile(**safety):
    return Profile(
        name="safety-test",
        sample_rate_hz=100.0,
        segments=[Segment(label="wot", throttle=1.0, duration_s=2.0)],
        safety=SafetyLimits(**safety),
    )


def _frame(current_a: float, temp_c: int = 30) -> KissFrame:
    return KissFrame(temperature_c=temp_c, voltage_v=16.0, current_a=current_a,
                     consumption_mah=0, e_rpm=10000, crc_ok=True)


def test_runner_trips_on_stand_current_without_backend_limits():
    # The stand itself gets NO limits (mirrors the gRPC backend, which cannot
    # enforce them until the vendor RPC is mapped) - the runner must trip.
    sim = RigSimulator(params=MotorParams(), noise=0.0)
    stand = SimulatedStand(sim, fixed_dt=0.01).open()
    sources = Sources(
        throttle=FlightStandThrottle(stand, arm_settle_s=0.0),
        stand=stand,
        perf_source=lambda: None,
        telem_source=lambda: None,
    )
    result = run_profile(_profile(max_current_a=5.0), sources, realtime=False)
    assert result.meta["aborted"] is not None
    assert "safety" in result.meta["aborted"]
    assert "current" in result.meta["aborted"]


def test_runner_trips_on_telemetry_only_rig():
    # No stand at all: the ESC telemetry current must still enforce the limit.
    class _DummyThrottle:
        def arm(self): pass
        def set(self, throttle): pass
        def disarm(self): pass
        def close(self): pass

    sources = Sources(
        throttle=_DummyThrottle(),
        stand=None,
        perf_source=lambda: None,
        telem_source=lambda: _frame(current_a=60.0),
    )
    result = run_profile(_profile(max_current_a=45.0), sources, realtime=False)
    assert result.meta["aborted"] is not None
    assert "current" in result.meta["aborted"]


def test_standless_run_completes_within_limits():
    class _DummyThrottle:
        def arm(self): pass
        def set(self, throttle): pass
        def disarm(self): pass
        def close(self): pass

    sources = Sources(
        throttle=_DummyThrottle(),
        stand=None,
        perf_source=lambda: None,
        telem_source=lambda: _frame(current_a=10.0),
    )
    result = run_profile(_profile(max_current_a=45.0), sources, realtime=False)
    assert result.meta["aborted"] is None
    assert len(result.rows) == 200
    assert result.rows[0]["stand_thrust_gf"] == ""  # stand columns empty


def test_runner_aborts_on_live_bemf_desync():
    """Once spin is established, firmware bemf_timeout must cut the run
    immediately — not finish the remaining high-throttle segments."""
    from hwci.perf import PerfSample

    class _DummyThrottle:
        def __init__(self):
            self.disarmed = False
            self.last = 0.0

        def arm(self): pass

        def set(self, throttle):
            self.last = throttle

        def disarm(self):
            self.disarmed = True

        def close(self): pass

    n = {"i": 0}

    def perf_source():
        n["i"] += 1
        # Establish spin for a few ticks, then latch bemf_timeout.
        if n["i"] < 30:
            raw = {"bemf_timeout_state": 0, "running": 1, "e_rpm": 50,
                   "ctrl_exec_us_last": 0, "ctrl_exec_us_max": 0,
                   "ctrl_period_us_last": 50, "ctrl_period_us_max": 50,
                   "ctrl_period_us_min": 50, "main_loop_us_last": 0,
                   "main_loop_us_max": 0, "input": 1000, "duty_cycle": 500,
                   "voltage_cv": 2200, "current_ca": 500, "temperature_c": 30,
                   "armed": 1, "loop_iters": 1, "zero_cross_count": 10,
                   "commutation_interval": 1000, "commutation_interval_max": 1000,
                   "update_count": n["i"], "host_cmd": 0, "magic": 1,
                   "version": 1, "size": 0}
        else:
            raw = {"bemf_timeout_state": 3, "running": 1, "e_rpm": 5,
                   "ctrl_exec_us_last": 0, "ctrl_exec_us_max": 0,
                   "ctrl_period_us_last": 50, "ctrl_period_us_max": 50,
                   "ctrl_period_us_min": 50, "main_loop_us_last": 0,
                   "main_loop_us_max": 0, "input": 1000, "duty_cycle": 500,
                   "voltage_cv": 2200, "current_ca": 5000, "temperature_c": 30,
                   "armed": 1, "loop_iters": 1, "zero_cross_count": 10,
                   "commutation_interval": 50000, "commutation_interval_max": 50000,
                   "update_count": n["i"], "host_cmd": 0, "magic": 1,
                   "version": 1, "size": 0}
        return PerfSample(raw=raw)

    thr = _DummyThrottle()
    sources = Sources(
        throttle=thr,
        stand=None,
        perf_source=perf_source,
        telem_source=lambda: None,
    )
    # 5 s would be 500 samples at 100 Hz; desync at tick 30 must abort early.
    result = run_profile(_profile(max_current_a=200.0), sources, realtime=False)
    assert result.meta["aborted"] is not None
    assert "desync" in result.meta["aborted"]
    assert "bemf_timeout" in result.meta["aborted"]
    assert thr.disarmed is True
    assert len(result.rows) < 200  # did not run the full 2 s segment out


def test_runner_aborts_on_rpm_collapse_while_throttle_high():
    from hwci.flightstand.base import StandSample

    class _DummyThrottle:
        def arm(self): pass
        def set(self, throttle): pass
        def disarm(self): pass
        def close(self): pass

    n = {"i": 0}

    def stand_read():
        n["i"] += 1
        rpm = 5000.0 if n["i"] < 50 else 1000.0  # collapse after peak
        return StandSample(t=0.0, throttle=1.0, thrust_n=5.0, torque_nm=0.0,
                           rpm=rpm, voltage_v=22.0, current_a=10.0)

    class _Stand:
        def read_sample(self):
            return stand_read()

    sources = Sources(
        throttle=_DummyThrottle(),
        stand=_Stand(),
        perf_source=lambda: None,
        telem_source=lambda: None,
    )
    result = run_profile(_profile(max_current_a=200.0), sources, realtime=False)
    assert result.meta["aborted"] is not None
    assert "desync" in result.meta["aborted"]
    assert "rpm collapse" in result.meta["aborted"]
    assert len(result.rows) < 200
