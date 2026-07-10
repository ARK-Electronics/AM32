'''DroneCAN RawCommand / telemetry path over multicast UDP.'''

from __future__ import annotations

import time

import pytest

from sitl_harness import rpm_from_state, wait_for_state

dronecan = pytest.importorskip('dronecan')


def _multicast_usable(sitl, sim, timeout=5.0):
    '''GitHub macOS (and some locked-down VMs) have no multicast route.
    Skip rather than fail when the SITL never starts streaming with CAN on.'''
    if wait_for_state(sim, timeout=timeout):
        return True
    pytest.skip(
        'SITL state stream never started with CAN enabled; '
        'multicast is probably unavailable.\n' + sitl.log_tail())


def test_dronecan_throttle_and_esc_status(sitl_can_factory, state_stream, mcast_uri):
    sitl = sitl_can_factory(
        extra_args=['--node-id', '10'],
        can_uri=mcast_uri,
        wait_s=1.0)
    sim = state_stream(sitl)
    _multicast_usable(sitl, sim)

    node = dronecan.make_node(mcast_uri, node_id=100, bitrate=1000000)
    status = {}

    def on_esc(e):
        status['rpm'] = e.message.rpm
        status['voltage'] = e.message.voltage
        status['count'] = status.get('count', 0) + 1

    node.add_handler(dronecan.uavcan.equipment.esc.Status, on_esc)
    try:
        t0 = time.time()
        nxt = t0
        while time.time() - t0 < 10:
            node.spin(0)
            now = time.time()
            if now >= nxt:
                nxt += 0.02
                thr = 0.35 if now - t0 > 2.5 else 0.0
                node.broadcast(
                    dronecan.uavcan.equipment.safety.ArmingStatus(status=255))
                node.broadcast(
                    dronecan.uavcan.equipment.esc.RawCommand(
                        cmd=[int(8191 * thr)]))
            time.sleep(0.001)

        rpm = rpm_from_state(sim)
        assert 3500 <= rpm <= 6500, 'state rpm=%.0f' % rpm
        assert 3500 <= status.get('rpm', -1) <= 6500, status
        assert 15 < status.get('voltage', 0) < 18, status
        assert status.get('count', 0) >= 3, 'too few esc.Status: %s' % status
    finally:
        # stop motor
        for _ in range(10):
            node.broadcast(dronecan.uavcan.equipment.esc.RawCommand(cmd=[0]))
            node.spin(0.02)
        node.close()


def test_dronecan_requires_arming(sitl_can_factory, state_stream, mcast_uri):
    '''default REQUIRE_ARMING=1: RawCommand alone must not spin the motor'''
    sitl = sitl_can_factory(
        extra_args=['--node-id', '10'],
        can_uri=mcast_uri,
        wait_s=1.0)
    sim = state_stream(sitl)
    _multicast_usable(sitl, sim)

    node = dronecan.make_node(mcast_uri, node_id=101, bitrate=1000000)
    try:
        t0 = time.time()
        nxt = t0
        while time.time() - t0 < 5.0:
            node.spin(0)
            now = time.time()
            if now >= nxt:
                nxt += 0.02
                # no ArmingStatus — only RawCommand at mid throttle
                node.broadcast(
                    dronecan.uavcan.equipment.esc.RawCommand(cmd=[4000]))
            time.sleep(0.001)
        rpm = rpm_from_state(sim, 0.5)
        assert rpm < 500, 'spun without arming: rpm=%.0f' % rpm
    finally:
        for _ in range(5):
            node.broadcast(dronecan.uavcan.equipment.esc.RawCommand(cmd=[0]))
            node.spin(0.02)
        node.close()
