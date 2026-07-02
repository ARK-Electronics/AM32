"""Px4Throttle: arm safety, deadman resends, rate limiting, shell quiesce.

Uses a fake client and an injected clock - no MAVLink, no sleeps.
"""
import pytest

from hwci.px4.client import (MAV_CMD_ACTUATOR_TEST, MAV_RESULT_ACCEPTED,
                             Px4Error)
from hwci.throttle.px4_src import Px4Throttle


class FakeClient:
    def __init__(self):
        self.armed = False
        self.calls = []          # (motor_index, value, timeout_s, wait_ack_s)
        self.shell_calls = []
        self.ack = MAV_RESULT_ACCEPTED
        self.arm_ack = MAV_RESULT_ACCEPTED

    def actuator_test(self, motor_index, value, *, timeout_s, wait_ack_s=None):
        self.calls.append((motor_index, value, timeout_s, wait_ack_s))
        return self.arm_ack if wait_ack_s is not None else None

    def last_ack(self, command):
        assert command == MAV_CMD_ACTUATOR_TEST
        return self.ack

    def shell(self, cmdline, **kw):
        self.shell_calls.append(cmdline)


class FakeClock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def make_throttle(**kw):
    client = FakeClient()
    clock = FakeClock()
    thr = Px4Throttle(client, clock=clock, sleep=lambda s: clock.advance(s),
                      **kw)
    return thr, client, clock


# --------------------------------------------------------------------------
# arm()
# --------------------------------------------------------------------------
def test_arm_sends_acked_zero():
    thr, client, _ = make_throttle(motor_index=3)
    thr.arm()
    motor, value, timeout, wait_ack = client.calls[0]
    assert (motor, value) == (3, 0.0)
    assert wait_ack is not None      # first command must be ACK'd
    assert timeout > 0               # FC-side deadman always set


def test_arm_refuses_when_vehicle_armed():
    thr, client, _ = make_throttle()
    client.armed = True
    with pytest.raises(Px4Error, match="ARMED"):
        thr.arm()
    assert client.calls == []        # never commanded anything


def test_arm_raises_on_rejected_ack():
    thr, client, _ = make_throttle()
    client.arm_ack = 4  # MAV_RESULT_FAILED
    with pytest.raises(Px4Error, match="rejected"):
        thr.arm()


# --------------------------------------------------------------------------
# set(): mapping, rate limiting, deadman refresh
# --------------------------------------------------------------------------
def test_set_clamps_to_unit_range():
    thr, client, clock = make_throttle()
    thr.set(1.7)
    clock.advance(1.0)
    thr.set(-0.3)
    values = [c[1] for c in client.calls]
    assert values == [1.0, 0.0]


def test_constant_throttle_resends_only_at_deadman_interval():
    thr, client, clock = make_throttle(resend_interval_s=0.25,
                                       cmd_timeout_s=1.0)
    for _ in range(10):              # 10 ticks inside one resend interval
        thr.set(0.5)
        clock.advance(0.01)
    assert len(client.calls) == 1
    clock.advance(0.25)
    thr.set(0.5)
    assert len(client.calls) == 2    # deadman refresh went out


def test_changed_value_is_rate_limited_not_flooded():
    thr, client, clock = make_throttle(min_interval_s=0.02)
    for i in range(10):              # 100 Hz ramp: value changes every tick
        thr.set(0.1 + i * 0.01)
        clock.advance(0.01)
    # 10 ticks over 0.1 s at a 0.02 s floor -> at most ~5 sends, not 10
    assert 1 < len(client.calls) <= 6


def test_deadman_timeout_always_exceeds_resend_interval():
    with pytest.raises(ValueError, match="deadman"):
        make_throttle(resend_interval_s=1.0, cmd_timeout_s=0.5)


def test_set_raises_when_fc_starts_rejecting():
    thr, client, clock = make_throttle()
    thr.set(0.4)
    client.ack = 2  # MAV_RESULT_DENIED
    clock.advance(1.0)
    with pytest.raises(Px4Error, match="rejecting"):
        thr.set(0.5)


# --------------------------------------------------------------------------
# disarm() / quiesce()
# --------------------------------------------------------------------------
def test_disarm_commands_zero_with_short_deadman():
    thr, client, _ = make_throttle()
    thr.disarm()
    motor, value, timeout, _ = client.calls[-1]
    assert value == 0.0
    assert timeout <= 0.5            # release quickly once resends stop


def test_disarm_swallows_link_errors():
    thr, client, _ = make_throttle()

    def boom(*a, **kw):
        raise RuntimeError("link gone")
    client.actuator_test = boom
    thr.disarm()                     # must not raise (runs in finally paths)


def test_quiesce_stops_dshot_and_arm_restarts_it():
    thr, client, _ = make_throttle()
    thr.quiesce()
    assert client.shell_calls == ["dshot stop"]
    thr.arm()
    assert client.shell_calls == ["dshot stop", "dshot start"]


def test_quiesce_without_shell_is_a_warning_not_a_stop(capsys):
    thr, client, _ = make_throttle(shell_quiesce=False)
    thr.quiesce()
    assert client.shell_calls == []
    assert "idle low" in capsys.readouterr().err
