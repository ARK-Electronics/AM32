'''Missed-ZC blind-step path tests (PR #41, BLHeli-style timeout commutation).

Uses state-port fault injection (cmd 3) to suppress comparator EXTI
delivery - the comparator LEVEL keeps tracking the physics, so poll mode
and the confirm loop stay honest while the interrupt path sees missed
crossings. Covers the three paths that previously rested only on bench
traces:

  - bridge:      a short full suppression is ridden out with blind steps,
                 no desync, no restart
  - limit:       a long full suppression exhausts the consecutive
                 blind-step budget and hands off to the stall rail
                 (restart), then recovers when the fault clears
  - alternating: dropping every other commutation window (the demag
                 signature) climbs the leaky miss-rate bucket until it
                 trips the same rail - the pattern the consecutive-step
                 counter alone can never catch
'''

from __future__ import annotations

import socket
import struct
import time

import sitl_dshot as sd
from sitl_harness import Sender, rpm_from_state, wait_for_state

STATE_MAGIC_CMD = 0x5353
ZC_STATS_MAGIC = 0x5356

STATS_FIELDS = ('zero_crosses', 'commutation_interval', 'dropped_edges',
                'desync_happened', 'old_routine', 'running', 'armed',
                'zc_blind_steps', 'zc_miss_bucket', 'zc_deadline_armed')
STATS_FMT = '<HBBIIIIBBBBBB'


def _open_ctl(sitl):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(('127.0.0.1', sitl.state_port))
    s.settimeout(0.5)
    return s


def _zc_fault(ctl, mode, duration_us):
    ctl.send(struct.pack('<HBBI', STATE_MAGIC_CMD, 3, mode, duration_us))


def _zc_stats(ctl, retries=5):
    for _ in range(retries):
        ctl.send(struct.pack('<HBB', STATE_MAGIC_CMD, 4, 0))
        try:
            pkt = ctl.recv(64)
        except socket.timeout:
            continue
        if len(pkt) >= struct.calcsize(STATS_FMT):
            vals = struct.unpack_from(STATS_FMT, pkt)
            if vals[0] == ZC_STATS_MAGIC:
                return dict(zip(STATS_FIELDS, vals[3:]))
    raise AssertionError('no ZC_STATS reply from SITL state port')


def _poll_stats(ctl, seconds):
    '''lockstep-sample stats as fast as the state port answers'''
    out = []
    deadline = time.time() + seconds
    while time.time() < deadline:
        out.append(_zc_stats(ctl))
    return out


def _spin_up(sitl, state_stream, value=250, arm_s=2.2, run_s=3.0):
    sim = state_stream(sitl)
    assert wait_for_state(sim), 'state stream dead\n' + sitl.log_tail()
    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT300)
    time.sleep(arm_s)
    tx.value = value
    time.sleep(run_s)
    rpm = rpm_from_state(sim)
    assert rpm > 500, 'motor not spinning: rpm=%.0f\n%s' % (rpm, sitl.log_tail())
    return sim, tx, rpm


def _assert_recovered(ctl, sim, rpm_before, sitl):
    time.sleep(1.5)
    end = _zc_stats(ctl)
    assert end['running'] == 1 and end['old_routine'] == 0, (
        'no closed-loop recovery: %r\n%s' % (end, sitl.log_tail()))
    assert end['zc_blind_steps'] == 0, end
    assert end['zc_miss_bucket'] < 6, end
    rpm = rpm_from_state(sim, 0.5)
    assert rpm > rpm_before * 0.6, (
        'rpm did not recover: %.0f vs %.0f before' % (rpm, rpm_before))


def test_blind_step_bridges_short_dropout(sitl_factory, state_stream):
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim, tx, rpm0 = _spin_up(sitl, state_stream)
    try:
        ctl = _open_ctl(sitl)
        pre = _zc_stats(ctl)
        assert pre['old_routine'] == 0 and pre['running'] == 1, pre
        ci_us = max(pre['commutation_interval'] // 2, 100)

        # ~3-4 missed crossings: well under the 8-step consecutive budget
        _zc_fault(ctl, mode=1, duration_us=6 * ci_us)
        samples = _poll_stats(ctl, seconds=max(0.2, 30 * ci_us * 1e-6))

        post = _zc_stats(ctl)
        assert post['dropped_edges'] > pre['dropped_edges'], (
            'fault gate never dropped an edge: %r -> %r' % (pre, post))
        # blind stepping engaged: the live counters were caught nonzero
        # (bucket lingers ~3 accepts per miss, so sampling cannot miss it)
        assert any(s['zc_blind_steps'] > 0 or s['zc_miss_bucket'] > 0
                   for s in samples), 'no blind step observed in %d samples' % len(samples)
        # and it BRIDGED: no restart (zero_crosses never reset), no desync,
        # never fell back to poll mode
        assert all(s['zero_crosses'] >= pre['zero_crosses'] for s in samples), (
            'zero_crosses reset during a bridgeable dropout')
        assert post['desync_happened'] == pre['desync_happened'], (pre, post)
        assert all(s['old_routine'] == 0 for s in samples)
        _assert_recovered(ctl, sim, rpm0, sitl)
    finally:
        tx.stop()


def test_blind_step_limit_hands_off_to_stall_rail(sitl_factory, state_stream):
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim, tx, rpm0 = _spin_up(sitl, state_stream)
    try:
        ctl = _open_ctl(sitl)
        pre = _zc_stats(ctl)
        assert pre['old_routine'] == 0 and pre['zero_crosses'] > 100, pre
        ci_us = max(pre['commutation_interval'] // 2, 100)

        # far beyond the 8-step budget (8 steps cost ~23x CI in total)
        _zc_fault(ctl, mode=1, duration_us=60 * ci_us)
        samples = _poll_stats(ctl, seconds=max(0.5, 120 * ci_us * 1e-6))

        # the limit tripped: stall-rail restart resets zero_crosses
        assert min(s['zero_crosses'] for s in samples) < 100, (
            'blind-step limit never handed off to the stall rail '
            '(min zc %d over %d samples)\n%s'
            % (min(s['zero_crosses'] for s in samples), len(samples),
               sitl.log_tail()))
        _assert_recovered(ctl, sim, rpm0, sitl)
    finally:
        tx.stop()


def test_alternating_misses_trip_miss_rate_bucket(sitl_factory, state_stream):
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim, tx, rpm0 = _spin_up(sitl, state_stream)
    try:
        ctl = _open_ctl(sitl)
        pre = _zc_stats(ctl)
        assert pre['old_routine'] == 0 and pre['zero_crosses'] > 100, pre

        # drop every other commutation window: real/missed alternation.
        # The consecutive counter resets on every accepted crossing, so
        # only the leaky bucket (+3 per miss, -1 per accept) can end this.
        _zc_fault(ctl, mode=2, duration_us=500_000)
        samples = _poll_stats(ctl, seconds=0.7)

        assert max(s['zc_miss_bucket'] for s in samples) >= 12, (
            'miss-rate bucket never climbed under sustained alternating '
            'misses (max %d over %d samples)'
            % (max(s['zc_miss_bucket'] for s in samples), len(samples)))
        assert min(s['zero_crosses'] for s in samples) < 100, (
            'bucket never tripped the stall rail: alternating misses ran '
            'unbounded\n' + sitl.log_tail())
        _assert_recovered(ctl, sim, rpm0, sitl)
    finally:
        tx.stop()
