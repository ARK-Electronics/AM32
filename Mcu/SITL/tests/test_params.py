'''DroneCAN parameter GetSet / save path.'''

from __future__ import annotations

import time

import pytest

from sitl_harness import wait_for_state

dronecan = pytest.importorskip('dronecan')


def _wait_for_node(uri, timeout=8.0, our_id=110):
    node = dronecan.make_node(uri, node_id=our_id, bitrate=1000000)
    found = {}

    def on_status(e):
        found[e.transfer.source_node_id] = True

    node.add_handler(dronecan.uavcan.protocol.NodeStatus, on_status)
    deadline = time.time() + timeout
    while time.time() < deadline and not found:
        node.spin(0.1)
    return node, found


def _request_wait(node, target, req, timeout=2.5):
    '''send a service request; pydronecan delivers timeout as e=None'''
    result = {}

    def cb(e):
        result['response'] = e.response if e is not None else None
        result['done'] = True

    node.request(req, target, cb)
    deadline = time.time() + timeout
    while 'done' not in result and time.time() < deadline:
        node.spin(0.05)
    return result.get('response')


def _get_param(node, target, name, attempts=5):
    '''GetSet with retries — idle SITL reboots every ~2s on signal timeout,
    which can swallow a single in-flight service transfer.'''
    for _ in range(attempts):
        req = dronecan.uavcan.protocol.param.GetSet.Request()
        req.name = name
        rsp = _request_wait(node, target, req)
        if rsp is not None and str(rsp.name):
            return rsp
        time.sleep(0.3)
    return None


def _set_param(node, target, name, value, attempts=5):
    for _ in range(attempts):
        req = dronecan.uavcan.protocol.param.GetSet.Request()
        req.name = name
        req.value = dronecan.uavcan.protocol.param.Value(integer_value=int(value))
        rsp = _request_wait(node, target, req)
        if rsp is not None and str(rsp.name):
            if int(rsp.value.integer_value) == int(value):
                return rsp
        time.sleep(0.3)
    return None


def test_param_get_defaults(sitl_factory, state_stream, mcast_uri):
    sitl = sitl_factory(
        extra_args=['--node-id', '10'],
        can_uri=mcast_uri,
        wait_s=1.0)
    sim = state_stream(sitl)
    if not wait_for_state(sim, timeout=5.0):
        pytest.skip('multicast unavailable\n' + sitl.log_tail())

    node, found = _wait_for_node(mcast_uri)
    try:
        assert 10 in found, 'ESC node 10 not seen: %s' % found
        rsp = _get_param(node, 10, 'MOTOR_POLES')
        assert rsp is not None, 'GetSet failed for MOTOR_POLES\n' + sitl.log_tail()
        assert int(rsp.value.integer_value) == 14, rsp.value
        rsp = _get_param(node, 10, 'INPUT_SIGNAL_TYPE')
        assert rsp is not None
        assert int(rsp.value.integer_value) in (0, 1, 2, 5), rsp.value
        rsp = _get_param(node, 10, 'TELEM_RATE')
        assert rsp is not None
        assert 0 <= int(rsp.value.integer_value) <= 200
    finally:
        node.close()


def test_param_set_telem_rate(sitl_factory, state_stream, mcast_uri):
    sitl = sitl_factory(
        extra_args=['--node-id', '10'],
        can_uri=mcast_uri,
        wait_s=1.0)
    sim = state_stream(sitl)
    if not wait_for_state(sim, timeout=5.0):
        pytest.skip('multicast unavailable\n' + sitl.log_tail())

    node, found = _wait_for_node(mcast_uri, our_id=111)
    try:
        assert 10 in found, found
        new_rate = 50
        rsp = _set_param(node, 10, 'TELEM_RATE', new_rate)
        assert rsp is not None, 'set TELEM_RATE failed\n' + sitl.log_tail()
        assert int(rsp.value.integer_value) == new_rate, rsp.value
        rsp = _get_param(node, 10, 'TELEM_RATE')
        assert rsp is not None
        assert int(rsp.value.integer_value) == new_rate
    finally:
        node.close()


def test_param_save_opcode(sitl_factory, state_stream, mcast_uri):
    sitl = sitl_factory(
        extra_args=['--node-id', '10'],
        can_uri=mcast_uri,
        wait_s=1.0)
    sim = state_stream(sitl)
    if not wait_for_state(sim, timeout=5.0):
        pytest.skip('multicast unavailable\n' + sitl.log_tail())

    node, found = _wait_for_node(mcast_uri, our_id=112)
    try:
        assert 10 in found, found
        assert _set_param(node, 10, 'BEEP_VOLUME', 7) is not None
        ok = False
        for _ in range(5):
            req = dronecan.uavcan.protocol.param.ExecuteOpcode.Request()
            req.opcode = req.OPCODE_SAVE
            rsp = _request_wait(node, 10, req, timeout=3.0)
            if rsp is not None and rsp.ok:
                ok = True
                break
            time.sleep(0.3)
        assert ok, 'OPCODE_SAVE failed\n' + sitl.log_tail()
    finally:
        node.close()
