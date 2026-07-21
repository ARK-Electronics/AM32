'''Safety / arming functional tests.

Firmware contract (Src/main.c, signal.c, DroneCAN.c):
  - DShot/PWM: arming needs ~1s of valid signal with zero throttle
    (adjusted_input == 0 and zero_input_count > 30). High throttle from
    boot must never spin the motor.
  - DroneCAN defaults REQUIRE_ARMING=1 and REQUIRE_ZERO_THROTTLE=1:
    high RawCommand without ArmingStatus must not spin; high throttle
    at first contact even with ArmingStatus must not spin until a zero
    throttle period has armed the ESC.
  - Bad / missing signal must not leave the motor running.
'''

from __future__ import annotations

import time

import pytest

import sitl_dshot as sd
from sitl_harness import Sender, rpm_from_state, wait_for_state


def _need_dronecan():
    return pytest.importorskip('dronecan')


def _assert_stopped(sim, label, window=0.5, limit=200.0):
    rpm = rpm_from_state(sim, window)
    assert rpm < limit, '%s: motor spinning rpm=%.0f (limit %.0f)' % (
        label, rpm, limit)


def _assert_spinning(sim, label, lo=2000.0, hi=20000.0, window=1.0):
    rpm = rpm_from_state(sim, window)
    assert lo <= rpm <= hi, '%s: rpm=%.0f expected %.0f..%.0f' % (
        label, rpm, lo, hi)


# ---------------------------------------------------------------------------
# DShot / PWM: boot with high throttle must not spin
# ---------------------------------------------------------------------------

def test_dshot_high_throttle_from_boot_does_not_spin(sitl_factory, state_stream):
    '''never send zero — high DShot from the first frame must not arm.'''
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim)
    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    try:
        tx.value = 1000  # mid-high throttle immediately, no zero period
        time.sleep(4.0)
        _assert_stopped(sim, 'dshot high-from-boot', window=0.8)
    finally:
        tx.stop()


def test_pwm_high_throttle_from_boot_does_not_spin(sitl_factory, state_stream):
    '''high PWM pulse from boot (no 1000 us arming) must not spin.'''
    sitl = sitl_factory(extra_args=['--input-type', '2'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim)
    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_PWM)
    try:
        tx.value = 1800  # high stick from first frame
        time.sleep(4.0)
        _assert_stopped(sim, 'pwm high-from-boot', window=0.8)
    finally:
        tx.stop()


def test_dshot_arms_only_after_zero_then_spins(sitl_factory, state_stream):
    '''high-from-boot blocked, then zero arms, then throttle spins, then stop.'''
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim)
    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    try:
        tx.value = 900
        time.sleep(2.5)
        _assert_stopped(sim, 'pre-arm high')

        tx.value = 0
        time.sleep(2.2)
        _assert_stopped(sim, 'arming zero')

        tx.value = 700
        time.sleep(3.5)
        _assert_spinning(sim, 'after proper arm', lo=3000, hi=9000)

        tx.value = 0
        time.sleep(3.0)
        _assert_stopped(sim, 'post-run zero', window=0.4, limit=500)
    finally:
        tx.stop()


def test_pwm_arms_only_after_low_then_spins(sitl_factory, state_stream):
    sitl = sitl_factory(extra_args=['--input-type', '2'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim)
    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_PWM)
    try:
        tx.value = 1600
        time.sleep(2.5)
        _assert_stopped(sim, 'pre-arm high pwm')

        tx.value = 1000
        time.sleep(2.2)
        _assert_stopped(sim, 'arming low pwm')

        tx.value = 1500
        time.sleep(3.5)
        _assert_spinning(sim, 'after pwm arm', lo=3500, hi=10000)

        tx.value = 1000
        time.sleep(3.0)
        _assert_stopped(sim, 'post-run low pwm', window=0.4, limit=500)
    finally:
        tx.stop()


# ---------------------------------------------------------------------------
# DroneCAN arming / zero-throttle safety
# ---------------------------------------------------------------------------

def _can_drive(dronecan, node, thr, armed=True, rate_hz=50.0, duration=3.0,
               send_arming_status=True):
    '''broadcast ArmingStatus + RawCommand for duration seconds.

    armed=True  → FULLY_ARMED (255)
    armed=False → SAFE (0) so REQUIRE_ARMING zeros the applied throttle;
                  merely stopping ArmingStatus traffic leaves the last
                  armed state latched in the ESC.
    '''
    t0 = time.time()
    nxt = t0
    period = 1.0 / rate_hz
    arm_status = 255 if armed else 0
    while time.time() - t0 < duration:
        node.spin(0)
        now = time.time()
        if now >= nxt:
            nxt += period
            if send_arming_status:
                node.broadcast(
                    dronecan.uavcan.equipment.safety.ArmingStatus(
                        status=arm_status))
            node.broadcast(
                dronecan.uavcan.equipment.esc.RawCommand(
                    cmd=[int(8191 * thr)]))
        time.sleep(0.001)


def test_dronecan_high_throttle_from_boot_does_not_spin(
        sitl_can_factory, state_stream, mcast_uri):
    '''REQUIRE_ZERO_THROTTLE=1: arm + high RawCommand from first contact
    must not spin until a zero-throttle arming window has passed.'''
    dronecan = _need_dronecan()
    sitl = sitl_can_factory(
        extra_args=['--node-id', '10'], can_uri=mcast_uri, wait_s=1.0)
    sim = state_stream(sitl)
    if not wait_for_state(sim, timeout=5.0):
        pytest.skip('multicast unavailable\n' + sitl.log_tail())

    node = dronecan.make_node(mcast_uri, node_id=120, bitrate=1000000)
    try:
        # no zero period — arm + 50% throttle immediately
        _can_drive(dronecan, node, thr=0.5, armed=True, duration=4.0)
        _assert_stopped(sim, 'can high-from-boot', window=0.8, limit=500)
    finally:
        _can_drive(dronecan, node, thr=0.0, armed=True, duration=0.5)
        node.close()


def test_dronecan_arms_after_zero_then_spins(
        sitl_can_factory, state_stream, mcast_uri):
    dronecan = _need_dronecan()
    sitl = sitl_can_factory(
        extra_args=['--node-id', '10'], can_uri=mcast_uri, wait_s=1.0)
    sim = state_stream(sitl)
    if not wait_for_state(sim, timeout=5.0):
        pytest.skip('multicast unavailable\n' + sitl.log_tail())

    node = dronecan.make_node(mcast_uri, node_id=121, bitrate=1000000)
    try:
        _can_drive(dronecan, node, thr=0.5, armed=True, duration=2.5)
        _assert_stopped(sim, 'can pre-arm high', limit=500)

        _can_drive(dronecan, node, thr=0.0, armed=True, duration=2.5)
        _assert_stopped(sim, 'can arming zero', limit=500)

        _can_drive(dronecan, node, thr=0.35, armed=True, duration=4.0)
        _assert_spinning(sim, 'can after proper arm', lo=3000, hi=8000)

        # Prop inertia coasts for a while after input goes to zero; match
        # the DShot post-run settle so CI hosts with variable load are not
        # flaky around ~800–1000 rpm residual.
        _can_drive(dronecan, node, thr=0.0, armed=True, duration=3.5)
        _assert_stopped(sim, 'can post-run zero', window=0.5, limit=1000)
    finally:
        _can_drive(dronecan, node, thr=0.0, armed=False, duration=0.3)
        node.close()


def test_dronecan_disarm_zeros_input(
        sitl_can_factory, state_stream, mcast_uri):
    '''dropping ArmingStatus while running must stop applying throttle
    (REQUIRE_ARMING=1 forces input to 0 when disarmed).'''
    dronecan = _need_dronecan()
    sitl = sitl_can_factory(
        extra_args=['--node-id', '10'], can_uri=mcast_uri, wait_s=1.0)
    sim = state_stream(sitl)
    if not wait_for_state(sim, timeout=5.0):
        pytest.skip('multicast unavailable\n' + sitl.log_tail())

    node = dronecan.make_node(mcast_uri, node_id=122, bitrate=1000000)
    try:
        _can_drive(dronecan, node, thr=0.0, armed=True, duration=2.5)
        _can_drive(dronecan, node, thr=0.35, armed=True, duration=4.0)
        _assert_spinning(sim, 'can spinning before disarm', lo=3000, hi=8000)

        # keep high RawCommand but publish SAFE ArmingStatus.
        # Prop inertia coasts after applied throttle is forced to zero; allow
        # the same residual band as post-run zero (CI saw ~800–900 at 3s).
        _can_drive(dronecan, node, thr=0.35, armed=False, duration=3.5)
        _assert_stopped(sim, 'can after SAFE arming status', window=0.6, limit=1000)
    finally:
        _can_drive(dronecan, node, thr=0.0, armed=False, duration=0.3)
        node.close()


# ---------------------------------------------------------------------------
# Signal loss / stop behaviour (DShot)
# ---------------------------------------------------------------------------

def test_dshot_signal_loss_stops_motor(sitl_factory, state_stream):
    '''after arming and spinning, ceasing frames must stop the motor
    (signal timeout → allOff / reset). Allow a reboot; omega must fall.'''
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim)
    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    try:
        time.sleep(2.2)
        tx.value = 700
        time.sleep(3.5)
        _assert_spinning(sim, 'before signal loss', lo=3000, hi=9000)

        tx.stop()  # stop sending entirely
        # armed timeout ~0.5s, then freewheel coast + possible reboot (~2s).
        # Prop inertia can leave >1–2k rpm after only 3s wall time on a
        # loaded CI host — wait for a full coast-down before asserting.
        time.sleep(5.0)
        _assert_stopped(sim, 'after signal loss', window=0.5, limit=800)
    finally:
        try:
            tx.stop()
        except Exception:
            pass
