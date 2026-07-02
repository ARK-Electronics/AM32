"""Throttle source that uses a PX4 flight controller's own DShot output.

For testing DShot and telemetry PRs: the Flight Stand's ESC output can't emit
bidirectional DShot or set the DShot telemetry bit, so those code paths need a
real flight controller (an ARK FPV running PX4 on the bench) as the signal
source. Throttle goes out as ``MAV_CMD_ACTUATOR_TEST`` on one motor output -
the same mechanism as QGroundControl's actuator sliders, which PX4 only
honours while DISARMED (exactly right for a bench).

Safety model:

* :meth:`arm` refuses to start if PX4 reports the vehicle armed, and requires
  the first actuator-test command to be ACK'd ACCEPTED before the run begins.
* Every command carries a short FC-side timeout (deadman): if the host stops
  resending - crash, unplugged USB - PX4 releases the output back to disarmed
  (DShot 0) by itself. :meth:`set` therefore re-sends at least every
  ``resend_interval_s`` even at constant throttle.
* Commands are rate-limited to one per ``min_interval_s`` so a 100-200 Hz
  sample loop ramping the throttle doesn't flood the MAVLink link.

Quiesce: PX4's dshot driver keeps emitting frames even when disarmed, so
"zero throttle" never lets the signal line idle low - which the AM32
bootloader needs at boot to jump to the app. :meth:`quiesce` stops the output
driver entirely over the MAVLink system shell (``dshot stop``); the next
:meth:`arm` restarts it.
"""
from __future__ import annotations

import sys
import time

from ..px4.client import (MAV_CMD_ACTUATOR_TEST, MAV_RESULT_ACCEPTED,
                          MAV_RESULT_IN_PROGRESS, Px4Error, Px4Mavlink)
from .base import ThrottleSource


class Px4Throttle(ThrottleSource):
    def __init__(self, client: Px4Mavlink, *, motor_index: int = 1,
                 arm_settle_s: float = 2.0,
                 min_interval_s: float = 0.02,
                 resend_interval_s: float = 0.25,
                 cmd_timeout_s: float = 1.0,
                 shell_quiesce: bool = True,
                 clock=time.monotonic, sleep=time.sleep):
        if not 1 <= motor_index <= 16:
            raise ValueError(f"motor_index {motor_index} not in 1..16")
        if cmd_timeout_s <= resend_interval_s:
            raise ValueError(
                "cmd_timeout_s must exceed resend_interval_s, or the FC-side "
                "deadman expires between routine resends and the motor stutters")
        self.client = client
        self.motor_index = motor_index
        self.arm_settle_s = arm_settle_s
        self.min_interval_s = min_interval_s
        self.resend_interval_s = resend_interval_s
        self.cmd_timeout_s = cmd_timeout_s
        self.shell_quiesce = shell_quiesce
        self._clock = clock
        self._sleep = sleep
        self._last_value: float | None = None
        self._last_sent_at = float("-inf")
        self._output_stopped = False

    def arm(self) -> None:
        if self.client.armed:
            raise Px4Error(
                "PX4 reports the vehicle ARMED - refusing to run an actuator "
                "test. Disarm the FC (and remove any RC/GCS arming source) "
                "before starting a bench run.")
        if self._output_stopped:
            # quiesce() stopped the dshot driver to let the signal line idle
            # low; bring it back before commanding throttle.
            self.client.shell("dshot start")
            self._sleep(1.0)
            self._output_stopped = False
        result = self.client.actuator_test(
            self.motor_index, 0.0, timeout_s=self.cmd_timeout_s,
            wait_ack_s=3.0)
        if result != MAV_RESULT_ACCEPTED:
            raise Px4Error(
                f"PX4 rejected the actuator test (COMMAND_ACK result "
                f"{result}). Check that the vehicle is disarmed, the safety "
                f"switch is off, and Motor {self.motor_index} is mapped to "
                "an output in the PX4 Actuators configuration.")
        self._mark_sent(0.0)
        # Hold DShot 0 long enough for AM32 to arm (it requires ~1 s of
        # sustained zero input).
        self._sleep(self.arm_settle_s)

    def set(self, throttle: float) -> None:
        value = max(0.0, min(1.0, throttle))
        now = self._clock()
        since = now - self._last_sent_at
        changed = self._last_value is None or abs(value - self._last_value) > 1e-4
        # Deadman refresh (resend_interval) OR a new value - but never faster
        # than min_interval, so ramps at the sample rate don't flood the link.
        if not ((changed and since >= self.min_interval_s)
                or since >= self.resend_interval_s):
            return
        ack = self.client.last_ack(MAV_CMD_ACTUATOR_TEST)
        if ack not in (None, MAV_RESULT_ACCEPTED, MAV_RESULT_IN_PROGRESS):
            raise Px4Error(
                f"PX4 started rejecting the actuator test mid-run "
                f"(COMMAND_ACK result {ack}) - the FC-side deadman has "
                "released the motor; aborting instead of sampling a dead "
                "output.")
        self.client.actuator_test(self.motor_index, value,
                                  timeout_s=self.cmd_timeout_s)
        self._mark_sent(value, at=now)

    def disarm(self) -> None:
        # Best-effort (runs in finally paths): command zero with a short
        # deadman and let it expire, releasing the output back to PX4's own
        # disarmed value (DShot 0).
        try:
            self.client.actuator_test(self.motor_index, 0.0, timeout_s=0.2)
        except Exception:
            pass
        self._mark_sent(0.0)

    def quiesce(self) -> None:
        self.disarm()
        if not self.shell_quiesce:
            print("px4 throttle: shell quiesce disabled - PX4 keeps emitting "
                  "DShot frames, so the signal line will NOT idle low and a "
                  "rebooting ESC may park in the AM32 bootloader",
                  file=sys.stderr)
            return
        self.client.shell("dshot stop")
        self._output_stopped = True
        self._sleep(0.5)

    def _mark_sent(self, value: float, at: float | None = None) -> None:
        self._last_value = value
        self._last_sent_at = self._clock() if at is None else at
