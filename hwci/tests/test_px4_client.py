"""PX4 MAVLink client: ESC_STATUS/ESC_INFO folding, KissFrame conversion,
COMMAND_ACK plumbing - all against a fake connection, no pymavlink needed."""
import queue
import time

import pytest

from hwci.px4.client import (MAV_CMD_ACTUATOR_TEST,
                             MAV_CMD_SET_MESSAGE_INTERVAL,
                             MAV_RESULT_ACCEPTED, EscReport, Px4Error,
                             Px4Mavlink)


class _Msg:
    """Duck-typed MAVLink message."""

    def __init__(self, mtype, src_component=1, **fields):
        self._mtype = mtype
        self._src_component = src_component
        self.__dict__.update(fields)

    def get_type(self):
        return self._mtype

    def get_srcComponent(self):
        return self._src_component


class _FakeMav:
    def __init__(self):
        self.commands = []       # (target_sys, target_comp, cmd, confirm, *p)
        self.serial_control = []  # (device, flags, timeout, baud, count, data)
        self.heartbeats = 0

    def command_long_send(self, *args):
        self.commands.append(args)

    def serial_control_send(self, *args):
        self.serial_control.append(args)

    def heartbeat_send(self, *args):
        self.heartbeats += 1


class _FakeConn:
    """recv_match feeds queued messages to the client's RX thread."""

    def __init__(self):
        self.mav = _FakeMav()
        self.target_system = 1
        self.target_component = 1
        self._queue = queue.Queue()
        self.closed = False

    def wait_heartbeat(self, timeout=None):
        return _Msg("HEARTBEAT", base_mode=0, autopilot=12, type=2)

    def push(self, msg):
        self._queue.put(msg)

    def recv_match(self, blocking=True, timeout=None):
        try:
            return self._queue.get(timeout=0.01)
        except queue.Empty:
            return None

    def close(self):
        self.closed = True


@pytest.fixture
def client():
    conn = _FakeConn()
    c = Px4Mavlink("fake:", conn=conn).open()
    yield c
    c.close()


def _wait_for(predicate, timeout_s=2.0):
    deadline = time.monotonic() + timeout_s
    while not predicate():
        if time.monotonic() > deadline:
            raise AssertionError("condition never became true")
        time.sleep(0.005)


def _push_esc_status(client, *, index=0, rpm=(6000, 0, 0, 0),
                     voltage=(23.9, 0, 0, 0), current=(4.2, 0, 0, 0)):
    client._conn.push(_Msg("ESC_STATUS", index=index, rpm=list(rpm),
                           voltage=list(voltage), current=list(current)))


# --------------------------------------------------------------------------
# ESC report folding + KissFrame conversion
# --------------------------------------------------------------------------
def test_esc_status_and_info_fold_into_one_report(client):
    _push_esc_status(client)
    client._conn.push(_Msg("ESC_INFO", index=0,
                           temperature=[4150, 0, 0, 0]))  # centi-degC
    _wait_for(lambda: (client.esc_report(0) or EscReport()).temperature_c
              is not None)
    r = client.esc_report(0)
    assert r.rpm == 6000
    assert r.voltage_v == pytest.approx(23.9)
    assert r.current_a == pytest.approx(4.2)
    assert r.temperature_c == pytest.approx(41.5)
    assert r.count == 1


def test_esc_frame_reconstructs_erpm_from_pole_pairs(client):
    # PX4 reports MECHANICAL rpm (it already divided by MOT_POLE_COUNT/2);
    # the harness stores eRPM, so the frame multiplies back.
    _push_esc_status(client, rpm=(6000, 0, 0, 0))
    _wait_for(lambda: client.esc_report(0) is not None)
    frame = client.esc_frame(0, pole_pairs=7)
    assert frame.e_rpm == 42000
    assert frame.mech_rpm(7) == pytest.approx(6000)
    assert frame.voltage_v == pytest.approx(23.9)
    assert frame.crc_ok  # synthesized, never a wire CRC failure


def test_esc_frame_none_when_stale_or_absent(client):
    assert client.esc_frame(0, pole_pairs=7) is None  # nothing received
    _push_esc_status(client)
    _wait_for(lambda: client.esc_report(0) is not None)
    with client._reports_lock:
        client._reports[0].t -= 10.0  # age the report past max_age
    assert client.esc_frame(0, pole_pairs=7, max_age_s=1.0) is None


def test_esc_status_index_offsets_multi_esc_messages(client):
    _push_esc_status(client, index=4, rpm=(1000, 2000, 3000, 4000))
    _wait_for(lambda: client.esc_report(7) is not None)
    assert client.esc_report(4).rpm == 1000
    assert client.esc_report(7).rpm == 4000
    assert client.esc_report(0) is None


# --------------------------------------------------------------------------
# Commands / ACKs / armed flag
# --------------------------------------------------------------------------
def test_actuator_test_sends_motor_function_and_deadman(client):
    client.actuator_test(2, 0.5, timeout_s=1.0)
    (_sys, _comp, cmd, _confirm, value, timeout, _p3, _p4,
     function, _p6, _p7) = client._conn.mav.commands[-1]
    assert cmd == MAV_CMD_ACTUATOR_TEST
    assert value == pytest.approx(0.5)
    assert timeout == pytest.approx(1.0)
    assert function == 2.0  # ACTUATOR_OUTPUT_FUNCTION_MOTOR1 + 1


def test_command_waits_for_ack(client):
    client._conn.push(_Msg("COMMAND_ACK", command=MAV_CMD_ACTUATOR_TEST,
                           result=MAV_RESULT_ACCEPTED))
    result = client.actuator_test(1, 0.0, timeout_s=1.0, wait_ack_s=2.0)
    assert result == MAV_RESULT_ACCEPTED
    assert client.last_ack(MAV_CMD_ACTUATOR_TEST) == MAV_RESULT_ACCEPTED


def test_command_ack_timeout_raises(client):
    with pytest.raises(Px4Error, match="COMMAND_ACK"):
        client.command(MAV_CMD_ACTUATOR_TEST, 0.0, wait_ack_s=0.05)


def test_armed_flag_tracks_autopilot_heartbeat_only(client):
    client._conn.push(_Msg("HEARTBEAT", src_component=190, base_mode=128))
    client._conn.push(_Msg("HEARTBEAT", src_component=1, base_mode=0))
    _wait_for(lambda: client._conn._queue.empty())
    assert client.armed is False  # component 190 (companion) must not count
    client._conn.push(_Msg("HEARTBEAT", src_component=1, base_mode=128))
    _wait_for(lambda: client.armed)


def test_request_esc_telemetry_raises_on_rejection(client):
    client._conn.push(_Msg("COMMAND_ACK",
                           command=MAV_CMD_SET_MESSAGE_INTERVAL, result=3))
    with pytest.raises(Px4Error, match="DSHOT_BIDIR_EN"):
        client.request_esc_telemetry(wait_ack_s=2.0)


def test_shell_sends_serial_control_chunks(client):
    client.shell("dshot stop", settle_s=0.0)
    (device, _flags, _timeout, _baud, count, data) = \
        client._conn.mav.serial_control[-1]
    assert device == 10  # SERIAL_CONTROL_DEV_SHELL
    assert count == len(b"dshot stop\n")
    assert bytes(data[:count]) == b"dshot stop\n"
    assert len(data) == 70


def test_close_stops_rx_and_closes_conn():
    conn = _FakeConn()
    c = Px4Mavlink("fake:", conn=conn).open()
    c.close()
    assert conn.closed
