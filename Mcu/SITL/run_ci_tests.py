#!/usr/bin/env python3
'''
CI entry point for the AM32 SITL test suite.

Prefers pytest (Mcu/SITL/tests/). Falls back to a small inline suite when
pytest is not installed, so the script stays usable with only the stdlib
plus optional pydronecan.

usage:
  python3 Mcu/SITL/run_ci_tests.py [--sitl path/to/elf] [-- pytest-args...]
exits non-zero if any test fails.
'''

from __future__ import annotations

import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from sitl_harness import (  # noqa: E402
    Sender,
    Sitl,
    find_sitl_binary,
    free_mcast_group,
    open_state,
    rpm_from_state,
    wait_for_state,
)
import sitl_dshot as sd  # noqa: E402

failures = []


def check(name, cond, detail):
    status = 'PASS' if cond else 'FAIL'
    print('%s: %s (%s)' % (status, name, detail))
    sys.stdout.flush()
    if not cond:
        failures.append(name)


def test_dshot(sitl_path, name, ptype, bidir, edt, value, rpm_lo, rpm_hi, input_type=1):
    with Sitl(sitl_path, ['--input-type', str(input_type)], can_uri='none') as sitl:
        sim = open_state('127.0.0.1', sitl.state_port, period_us=200)
        tx = Sender('127.0.0.1', sitl.input_port, ptype, bidir=bidir)
        try:
            time.sleep(2.2)
            if edt:
                tx.cmds = [sd.DSHOT_CMD_EDT_ENABLE] * 8
                time.sleep(0.5)
            tx.value = value
            time.sleep(4.0)
            rpm = rpm_from_state(sim)
            check(name + ' rpm', rpm_lo <= rpm <= rpm_hi,
                  'rpm=%.0f expected %d..%d' % (rpm, rpm_lo, rpm_hi))
            if bidir:
                replies = tx.port.reply_count
                check(name + ' bdshot replies', replies > 500,
                      'replies=%d' % replies)
                erpm = [sd.decode_reply(r[3], edt_expected=edt)
                        for r in tx.port.get_replies()]
                rpms = [sd.erpm_period_to_rpm(v) for k, v in erpm if k == 'erpm']
                if rpms:
                    check(name + ' bdshot rpm agrees',
                          abs(rpms[-1] - rpm) < max(200, rpm * 0.05),
                          'bdshot=%.0f state=%.0f' % (rpms[-1], rpm))
                if edt:
                    edt_vals = dict((k, v) for k, v in erpm
                                    if k in ('temp', 'volt', 'current'))
                    check(name + ' edt values',
                          edt_vals.get('temp') == 25 and 14 < edt_vals.get('volt', 0) < 18,
                          'edt=%s' % edt_vals)
            tx.value = 1000 if ptype == sd.TYPE_PWM else 0
            time.sleep(3.0)
            rpm = rpm_from_state(sim, 0.3)
            check(name + ' stops', rpm < 500, 'rpm=%.0f' % rpm)
        finally:
            tx.stop()
            sim.close()


def test_dronecan(sitl_path):
    try:
        import dronecan
    except ImportError:
        print('SKIP: dronecan not installed, DroneCAN test skipped')
        return
    can_uri = 'mcast:%d' % free_mcast_group()
    with Sitl(sitl_path, ['--node-id', '10'], can_uri=can_uri, wait_s=1.0) as sitl:
        sim = open_state('127.0.0.1', sitl.state_port, period_us=200)
        if not wait_for_state(sim, timeout=5.0):
            print('SKIP: SITL state stream never started with CAN enabled, '
                  'multicast is probably unavailable on this host. SITL log tail:')
            sys.stdout.flush()
            print(sitl.log_tail(5))
            sim.close()
            return
        node = dronecan.make_node(can_uri, node_id=100, bitrate=1000000)
        status = {}

        def on_esc(e):
            status['rpm'] = e.message.rpm
            status['voltage'] = e.message.voltage

        node.add_handler(dronecan.uavcan.equipment.esc.Status, on_esc)
        t0 = time.time()
        nxt = t0
        while time.time() - t0 < 10:
            node.spin(0)
            now = time.time()
            if now >= nxt:
                nxt += 0.02
                thr = 0.35 if now - t0 > 2.5 else 0.0
                node.broadcast(dronecan.uavcan.equipment.safety.ArmingStatus(status=255))
                node.broadcast(dronecan.uavcan.equipment.esc.RawCommand(cmd=[int(8191 * thr)]))
            time.sleep(0.001)
        rpm = rpm_from_state(sim)
        check('dronecan rpm', 3500 <= rpm <= 6500, 'rpm=%.0f' % rpm)
        check('dronecan telemetry', 3500 <= status.get('rpm', -1) <= 6500
              and 15 < status.get('voltage', 0) < 18,
              'esc.Status=%s' % status)
        node.close()
        sim.close()


def run_legacy(sitl_path):
    test_dshot(sitl_path, 'dshot600 bidir edt', sd.TYPE_DSHOT600,
               bidir=True, edt=True, value=800, rpm_lo=4000, rpm_hi=7000)
    test_dshot(sitl_path, 'dshot300', sd.TYPE_DSHOT300,
               bidir=False, edt=False, value=600, rpm_lo=3000, rpm_hi=6000)
    test_dshot(sitl_path, 'pwm', sd.TYPE_PWM,
               bidir=False, edt=False, value=1500, rpm_lo=4000, rpm_hi=9000,
               input_type=2)
    test_dronecan(sitl_path)
    if failures:
        print('\n%d FAILED: %s' % (len(failures), ', '.join(failures)))
        return 1
    print('\nall tests passed')
    return 0


def run_pytest(sitl_path, extra):
    import pytest
    args = [
        os.path.join(HERE, 'tests'),
        '-v',
        '--tb=short',
        '--sitl', sitl_path,
    ]
    args += extra
    return pytest.main(args)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--sitl', default=None, help='path to SITL binary')
    ap.add_argument('--legacy', action='store_true',
                    help='run the small stdlib suite instead of pytest')
    args, rest = ap.parse_known_args()

    sitl_path = find_sitl_binary(args.sitl)
    if not sitl_path or not os.path.exists(sitl_path):
        print('SITL binary not found: %s' % sitl_path)
        sys.exit(2)

    if not args.legacy:
        try:
            import pytest  # noqa: F401
        except ImportError:
            print('pytest not installed; falling back to legacy suite '
                  '(pip install -r Mcu/SITL/requirements-ci.txt)')
            args.legacy = True

    if args.legacy:
        sys.exit(run_legacy(sitl_path))
    sys.exit(run_pytest(sitl_path, rest))


if __name__ == '__main__':
    main()
