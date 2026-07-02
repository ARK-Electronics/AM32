"""build_live_sources wiring for the px4 backends: one shared MAVLink client
feeds both the throttle source and the telemetry channel."""
import pytest

from hwci.config import RigConfig, load_profile
from hwci.esc_telem.kiss import KissFrame
from hwci.px4.client import MAV_RESULT_ACCEPTED
from hwci.runner import build_live_sources
from hwci.throttle.px4_src import Px4Throttle


class FakePx4Client:
    instances: list = []

    def __init__(self, url, baud):
        self.url = url
        self.baud = baud
        self.opened = False
        self.closed = False
        self.telem_requested = False
        self.armed = False
        self.esc_frame_calls = []
        FakePx4Client.instances.append(self)

    def open(self):
        self.opened = True
        return self

    def close(self):
        self.closed = True

    def request_esc_telemetry(self, **kw):
        self.telem_requested = True

    def esc_frame(self, esc_index, pole_pairs, max_age_s=1.0):
        self.esc_frame_calls.append((esc_index, pole_pairs))
        return KissFrame(temperature_c=30, voltage_v=23.7, current_a=1.2,
                         consumption_mah=0, e_rpm=7000, crc_ok=True)

    def actuator_test(self, *a, **kw):
        return MAV_RESULT_ACCEPTED

    def last_ack(self, command):
        return MAV_RESULT_ACCEPTED

    def shell(self, cmdline, **kw):
        pass


@pytest.fixture
def px4_rig(monkeypatch):
    import hwci.px4.client as clientmod
    FakePx4Client.instances = []
    monkeypatch.setattr(clientmod, "Px4Mavlink", FakePx4Client)
    return RigConfig(debugger_backend="none", telem_backend="px4",
                     throttle_backend="px4", stand_backend="none",
                     px4_url="/dev/ttyTEST", px4_motor_index=2, pole_pairs=7)


def test_px4_sources_share_one_client(px4_rig):
    profile = load_profile("noprop_smoke")
    sources = build_live_sources(px4_rig, profile, tare=False)
    try:
        assert len(FakePx4Client.instances) == 1  # throttle + telem share it
        client = FakePx4Client.instances[0]
        assert client.opened and client.telem_requested
        assert client.url == "/dev/ttyTEST"
        assert isinstance(sources.throttle, Px4Throttle)
        assert sources.throttle.motor_index == 2
        assert sources.stand is None
        frame = sources.telem_source()
        assert frame.e_rpm == 7000
        # esc index defaults to motor_index - 1; pole_pairs from the rig
        assert client.esc_frame_calls[-1] == (1, 7)
    finally:
        sources.close()
    assert FakePx4Client.instances[0].closed


def test_px4_telem_without_px4_throttle_is_rejected(px4_rig):
    px4_rig.throttle_backend = "external"
    with pytest.raises(ValueError, match="telem_backend 'px4'"):
        build_live_sources(px4_rig, load_profile("noprop_smoke"), tare=False)