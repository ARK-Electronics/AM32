'''PWM/DShot input path tests against the SITL motor model.'''

from __future__ import annotations

import time

import pytest

import sitl_dshot as sd
from sitl_harness import Sender, rpm_from_state, wait_for_state


def _run_throttle(sitl, state_stream, ptype, value, rpm_lo, rpm_hi,
                  bidir=False, edt=False, arm_s=2.2, run_s=4.0, stop_s=3.0):
    sim = state_stream(sitl)
    assert wait_for_state(sim), 'state stream dead\n' + sitl.log_tail()
    tx = Sender('127.0.0.1', sitl.input_port, ptype, bidir=bidir)
    try:
        time.sleep(arm_s)
        if edt:
            tx.cmds = [sd.DSHOT_CMD_EDT_ENABLE] * 8
            time.sleep(0.5)
        tx.value = value
        time.sleep(run_s)
        rpm = rpm_from_state(sim)
        assert rpm_lo <= rpm <= rpm_hi, (
            'rpm=%.0f expected %d..%d\n%s' % (rpm, rpm_lo, rpm_hi, sitl.log_tail()))

        if bidir:
            replies = tx.port.reply_count
            assert replies > 500, 'too few BDShot replies: %d' % replies
            erpm = [sd.decode_reply(r[3], edt_expected=edt)
                    for r in tx.port.get_replies()]
            rpms = [sd.erpm_period_to_rpm(v) for k, v in erpm if k == 'erpm']
            if rpms:
                assert abs(rpms[-1] - rpm) < max(200, rpm * 0.05), (
                    'bdshot=%.0f state=%.0f' % (rpms[-1], rpm))
            if edt:
                edt_vals = {k: v for k, v in erpm
                            if k in ('temp', 'volt', 'current')}
                assert edt_vals.get('temp') == 25, edt_vals
                assert 14 < edt_vals.get('volt', 0) < 18, edt_vals

        # must stop again at zero throttle
        tx.value = 0
        time.sleep(stop_s)
        rpm = rpm_from_state(sim, 0.3)
        assert rpm < 500, 'motor did not stop: rpm=%.0f' % rpm
    finally:
        tx.stop()


def test_dshot600_bidir_edt(sitl_factory, state_stream):
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    _run_throttle(sitl, state_stream, sd.TYPE_DSHOT600, value=800,
                  rpm_lo=4000, rpm_hi=7000, bidir=True, edt=True)


def test_dshot300(sitl_factory, state_stream):
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    _run_throttle(sitl, state_stream, sd.TYPE_DSHOT300, value=600,
                  rpm_lo=3000, rpm_hi=6000)


def test_zero_throttle_stays_stopped(sitl_factory, state_stream):
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim)
    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600, bidir=True)
    try:
        tx.value = 0
        time.sleep(3.0)
        rpm = rpm_from_state(sim, 0.5)
        assert rpm < 200, 'spun at zero throttle: rpm=%.0f' % rpm
    finally:
        tx.stop()


def test_bad_crc_frames_do_not_arm_or_spin(sitl_factory, state_stream):
    '''all-corrupted frames must never produce throttle'''
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim)
    port = sd.InputPort('127.0.0.1', sitl.input_port)
    try:
        t0 = time.time()
        nxt = t0
        while time.time() - t0 < 4.0:
            now = time.time()
            while now >= nxt:
                nxt += 1.0 / 500.0
                port.send_dshot(800, ptype=sd.TYPE_DSHOT600, corrupt=True)
            time.sleep(0.0005)
        rpm = rpm_from_state(sim, 0.5)
        assert rpm < 200, 'motor spun on bad-CRC frames: rpm=%.0f' % rpm
    finally:
        port.close()
