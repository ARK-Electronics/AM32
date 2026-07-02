"""MAVLink client for a PX4 flight controller used as the ESC signal source.

The Tyto Flight Stand's ESC output can emit plain PWM/DShot but not
BIDIRECTIONAL DShot, and it never sets the DShot telemetry bit - so it cannot
exercise the AM32 code paths a DShot or telemetry PR changes. A real flight
controller can: an ARK FPV running PX4 drives the ESC with genuine
(bidirectional) DShot from its own dshot driver, requests KISS serial
telemetry over the telemetry wire, and reports everything it hears back from
the ESC over MAVLink (``ESC_STATUS``/``ESC_INFO``).

This module is the shared MAVLink transport for both hwci backends built on
it:

* :class:`~hwci.throttle.px4_src.Px4Throttle` (``throttle_backend: px4``) -
  commands per-motor throttle with ``MAV_CMD_ACTUATOR_TEST`` (the same
  mechanism as QGroundControl's actuator sliders; works only while DISARMED,
  which is exactly what a bench wants).
* the ``telem_backend: px4`` source - folds ``ESC_STATUS`` (eRPM/V/A, from
  bidir DShot and/or DShot-requested KISS telemetry) and ``ESC_INFO``
  (temperature) into :class:`~hwci.esc_telem.kiss.KissFrame` rows, so the
  round trip ESC -> FC -> host is what gets graded - a telemetry PR that
  breaks the wire format shows up as a dead ``telem_coverage`` channel.

pymavlink is imported lazily in :meth:`Px4Mavlink.open` (only needed on the
rig); tests inject a fake connection object instead.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field

from ..esc_telem.kiss import KissFrame

# MAVLink constants (common.xml), spelled out so importing this module never
# requires pymavlink.
MAV_CMD_ACTUATOR_TEST = 310
MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAVLINK_MSG_ID_ESC_INFO = 290
MAVLINK_MSG_ID_ESC_STATUS = 291
ACTUATOR_OUTPUT_FUNCTION_MOTOR1 = 1
MAV_RESULT_ACCEPTED = 0
MAV_RESULT_IN_PROGRESS = 5
MAV_MODE_FLAG_SAFETY_ARMED = 128
MAV_TYPE_GCS = 6
MAV_AUTOPILOT_INVALID = 8
MAV_COMP_ID_AUTOPILOT1 = 1
SERIAL_CONTROL_DEV_SHELL = 10
SERIAL_CONTROL_FLAG_RESPOND = 2
SERIAL_CONTROL_FLAG_EXCLUSIVE = 4
_SERIAL_CONTROL_CHUNK = 70  # data[] size of the SERIAL_CONTROL message


class Px4Error(RuntimeError):
    """The flight controller refused, or never answered, a command."""


@dataclass
class EscReport:
    """Most recent per-ESC data folded from ESC_STATUS / ESC_INFO."""
    t: float = float("-inf")   # host monotonic of the last ESC_STATUS update
    rpm: float | None = None   # mechanical RPM as PX4 reports it (MOT_POLE_COUNT)
    voltage_v: float | None = None
    current_a: float | None = None
    temperature_c: float | None = None
    count: int = 0             # ESC_STATUS messages folded in


class Px4Mavlink:
    """Thin, thread-safe MAVLink session to one PX4 autopilot.

    A background RX thread keeps the armed flag, COMMAND_ACKs and per-ESC
    reports current (and emits our own 1 Hz GCS heartbeat). Everything the
    sample loop touches is a dictionary lookup - no MAVLink round trip on the
    tick path.
    """

    def __init__(self, url: str, baud: int = 115200, *,
                 heartbeat_timeout_s: float = 10.0, conn=None):
        self.url = url
        self.baud = baud
        self.heartbeat_timeout_s = heartbeat_timeout_s
        self._conn = conn                 # tests inject a fake here
        self.target_system = 1
        self.target_component = MAV_COMP_ID_AUTOPILOT1
        self._armed = False
        self._acks: dict[int, int] = {}   # command id -> last COMMAND_ACK result
        self._ack_cond = threading.Condition()
        self._reports: dict[int, EscReport] = {}
        self._reports_lock = threading.Lock()
        self._rx_errors = 0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    # -- lifecycle ----------------------------------------------------------
    def open(self) -> "Px4Mavlink":
        if self._conn is None:
            from pymavlink import mavutil  # only needed on the rig
            self._conn = mavutil.mavlink_connection(
                self.url, baud=self.baud, source_system=254)
        hb = self._conn.wait_heartbeat(timeout=self.heartbeat_timeout_s)
        if hb is None:
            raise Px4Error(
                f"no MAVLink heartbeat from {self.url} within "
                f"{self.heartbeat_timeout_s:.0f}s - is the FC connected and "
                "powered, and the URL/baud right?")
        self.target_system = getattr(self._conn, "target_system", 1) or 1
        self._thread = threading.Thread(target=self._rx_loop, daemon=True,
                                        name="px4-mavlink-rx")
        self._thread.start()
        return self

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._conn is not None:
            try:
                self._conn.close()
            except Exception:
                pass
            self._conn = None

    # -- RX side ------------------------------------------------------------
    def _rx_loop(self) -> None:
        next_heartbeat = 0.0
        while not self._stop.is_set():
            now = time.monotonic()
            if now >= next_heartbeat:
                try:
                    self._conn.mav.heartbeat_send(
                        MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, 0)
                except Exception:
                    pass
                next_heartbeat = now + 1.0
            try:
                msg = self._conn.recv_match(blocking=True, timeout=0.2)
            except Exception:
                self._rx_errors += 1
                continue
            if msg is not None:
                self._handle(msg)

    def _handle(self, msg) -> None:
        mtype = msg.get_type()
        if mtype == "HEARTBEAT":
            # Only the autopilot's own heartbeat carries the armed flag
            # (onboard computers/cameras heartbeat too).
            if msg.get_srcComponent() == MAV_COMP_ID_AUTOPILOT1:
                self._armed = bool(msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
        elif mtype == "COMMAND_ACK":
            with self._ack_cond:
                self._acks[msg.command] = msg.result
                self._ack_cond.notify_all()
        elif mtype == "ESC_STATUS":
            now = time.monotonic()
            with self._reports_lock:
                for i in range(len(msg.rpm)):
                    r = self._reports.setdefault(msg.index + i, EscReport())
                    r.rpm = float(msg.rpm[i])
                    r.voltage_v = float(msg.voltage[i])
                    r.current_a = float(msg.current[i])
                    r.t = now
                    r.count += 1
        elif mtype == "ESC_INFO":
            with self._reports_lock:
                for i in range(len(msg.temperature)):
                    r = self._reports.setdefault(msg.index + i, EscReport())
                    r.temperature_c = msg.temperature[i] / 100.0  # cdegC

    @property
    def armed(self) -> bool:
        return self._armed

    @property
    def rx_errors(self) -> int:
        return self._rx_errors

    # -- TX side ------------------------------------------------------------
    def command(self, command: int, *params: float,
                wait_ack_s: float | None = None) -> int | None:
        """Send a COMMAND_LONG. With ``wait_ack_s``, block for the ACK and
        return its result; without, fire and forget (the RX thread still
        records the ACK for :meth:`last_ack`)."""
        p = list(params) + [0.0] * (7 - len(params))
        if wait_ack_s is not None:
            with self._ack_cond:
                self._acks.pop(command, None)  # await a FRESH ack, not a stale one
        self._conn.mav.command_long_send(
            self.target_system, self.target_component, command, 0, *p)
        if wait_ack_s is None:
            return None
        deadline = time.monotonic() + wait_ack_s
        with self._ack_cond:
            while command not in self._acks:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise Px4Error(
                        f"no COMMAND_ACK for MAVLink command {command} within "
                        f"{wait_ack_s:.1f}s")
                self._ack_cond.wait(remaining)
            return self._acks[command]

    def last_ack(self, command: int) -> int | None:
        """Most recent COMMAND_ACK result for ``command`` (None if none seen)."""
        with self._ack_cond:
            return self._acks.get(command)

    def actuator_test(self, motor_index: int, value: float, *,
                      timeout_s: float,
                      wait_ack_s: float | None = None) -> int | None:
        """MAV_CMD_ACTUATOR_TEST on MOTORn (1-based). ``value`` is 0..1 for a
        motor; ``timeout_s`` is the FC-side deadman - if the host stops
        resending, PX4 releases the output back to disarmed by itself."""
        function = ACTUATOR_OUTPUT_FUNCTION_MOTOR1 + (motor_index - 1)
        return self.command(MAV_CMD_ACTUATOR_TEST,
                            float(value), float(timeout_s), 0.0, 0.0,
                            float(function),
                            wait_ack_s=wait_ack_s)

    def request_esc_telemetry(self, *, status_rate_hz: float = 20.0,
                              info_rate_hz: float = 2.0,
                              wait_ack_s: float = 3.0) -> None:
        """Ask PX4 to stream ESC_STATUS/ESC_INFO at a useful rate. Fails loud:
        a rejected interval request means this PX4 has no ESC telemetry to
        stream (bidir DShot / DSHOT_TEL_CFG not configured), and the run's
        telemetry channel would silently stay empty."""
        for msg_id, rate in ((MAVLINK_MSG_ID_ESC_STATUS, status_rate_hz),
                             (MAVLINK_MSG_ID_ESC_INFO, info_rate_hz)):
            result = self.command(MAV_CMD_SET_MESSAGE_INTERVAL,
                                  float(msg_id), 1e6 / rate,
                                  wait_ack_s=wait_ack_s)
            if result not in (MAV_RESULT_ACCEPTED, MAV_RESULT_IN_PROGRESS):
                raise Px4Error(
                    f"PX4 rejected SET_MESSAGE_INTERVAL for msg {msg_id} "
                    f"(COMMAND_ACK result {result}). ESC telemetry is not "
                    "available on this FC - enable bidirectional DShot "
                    "(DSHOT_BIDIR_EN=1) and/or the ESC telemetry UART "
                    "(DSHOT_TEL_CFG) and reboot the FC.")

    def shell(self, cmdline: str, *, settle_s: float = 0.5) -> None:
        """Run one line on the PX4 system console (MAVLink SERIAL_CONTROL
        shell). Fire and forget - used for `dshot stop`/`dshot start` where
        the observable effect (signal line drops / returns) is verified by
        the caller's own bring-up logic, not by parsing console output."""
        payload = (cmdline.rstrip("\n") + "\n").encode()
        for off in range(0, len(payload), _SERIAL_CONTROL_CHUNK):
            chunk = payload[off:off + _SERIAL_CONTROL_CHUNK]
            data = list(chunk) + [0] * (_SERIAL_CONTROL_CHUNK - len(chunk))
            self._conn.mav.serial_control_send(
                SERIAL_CONTROL_DEV_SHELL,
                SERIAL_CONTROL_FLAG_RESPOND | SERIAL_CONTROL_FLAG_EXCLUSIVE,
                0, 0, len(chunk), data)
        time.sleep(settle_s)

    # -- ESC telemetry view ---------------------------------------------------
    def esc_report(self, esc_index: int,
                   max_age_s: float = 1.0) -> EscReport | None:
        """Latest report for one ESC, or None once it has gone stale (a stale
        sample repeated forever would defeat the telemetry-coverage gate)."""
        with self._reports_lock:
            r = self._reports.get(esc_index)
            if r is None or time.monotonic() - r.t > max_age_s:
                return None
            return EscReport(t=r.t, rpm=r.rpm, voltage_v=r.voltage_v,
                             current_a=r.current_a,
                             temperature_c=r.temperature_c, count=r.count)

    def esc_frame(self, esc_index: int, pole_pairs: int,
                  max_age_s: float = 1.0) -> KissFrame | None:
        """The ESC report as a KissFrame row.

        PX4 reports MECHANICAL rpm (it divides eRPM by MOT_POLE_COUNT/2), and
        the harness stores eRPM, so this multiplies back by ``pole_pairs`` -
        the FC's MOT_POLE_COUNT must equal 2 * the rig's ``pole_pairs`` or
        both rpm figures are silently wrong. Consumption is not reported over
        ESC_STATUS and is always 0.
        """
        r = self.esc_report(esc_index, max_age_s)
        if r is None:
            return None
        return KissFrame(
            temperature_c=int(round(r.temperature_c))
            if r.temperature_c is not None else 0,
            voltage_v=r.voltage_v or 0.0,
            current_a=r.current_a or 0.0,
            consumption_mah=0,
            e_rpm=int(round((r.rpm or 0.0) * pole_pairs)),
            crc_ok=True,
        )
