#!/usr/bin/env python3
'''
CI test runner for the AM32 SITL: starts the simulator, drives it
through the PWM/DShot and DroneCAN input paths and asserts on the
results. Only needs the python standard library; the DroneCAN test runs
when the dronecan package is importable and is skipped otherwise.

usage: run_ci_tests.py [--sitl path/to/elf]
exits non-zero if any test fails.
'''

import argparse
import os
import struct
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sitl_dshot as sd
from sitl_gui_backend import SimStream

HERE = os.path.dirname(os.path.abspath(__file__))
INPUT_PORT = 57833
STATE_PORT = 57834
CAN_URI = 'mcast:7'

failures = []


def check(name, cond, detail):
    status = 'PASS' if cond else 'FAIL'
    print('%s: %s (%s)' % (status, name, detail))
    sys.stdout.flush()
    if not cond:
        failures.append(name)


class Sitl(object):
    def __init__(self, sitl_path, extra_args=()):
        args = [sitl_path, '--input-port', str(INPUT_PORT),
                '--state-port', str(STATE_PORT), '--nosleep'] + list(extra_args)
        self.log = open('sitl_ci.log', 'ab')
        self.proc = subprocess.Popen(args, stdout=self.log, stderr=self.log)
        time.sleep(0.5)

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.proc.kill()
        self.proc.wait()
        self.log.close()
        for f in os.listdir('.'):
            if f.startswith('am32_eeprom.bin'):
                os.unlink(f)


class Sender(object):
    '''background frame sender with rate catch-up, like the GUI'''

    def __init__(self, ptype, bidir=False, rate=500.0):
        self.port = sd.InputPort('127.0.0.1', INPUT_PORT)
        self.ptype = ptype
        self.bidir = bidir
        self.rate = rate
        self.value = 1000 if ptype == sd.TYPE_PWM else 0
        self.cmds = []
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        nxt = time.time()
        try:
            self._run()
        except OSError:
            pass  # socket closed by stop()

    def _run(self):
        nxt = time.time()
        while self.running:
            now = time.time()
            burst = 0
            while now >= nxt and burst < 10:
                nxt += 1.0 / self.rate
                if self.cmds:
                    self.port.send_dshot(self.cmds.pop(0), ptype=self.ptype,
                                         telem=True, bidir=self.bidir)
                elif self.ptype == sd.TYPE_PWM:
                    self.port.send_pwm(int(self.value))
                else:
                    self.port.send_dshot(int(self.value), ptype=self.ptype,
                                         bidir=self.bidir)
                burst += 1
            if now - nxt > 0.25:
                nxt = now
            time.sleep(0.0005)

    def stop(self):
        self.running = False
        self.port.close()


def rpm_from_state(sim, window=1.0):
    w = sim.window(window)
    if not w:
        return -1
    return sum(s[1] for s in w) / len(w) * 60.0 / 6.28318


def test_dshot(sitl_path, name, ptype, bidir, edt, value, rpm_lo, rpm_hi, input_type=1):
    with Sitl(sitl_path, ['--can-uri', 'none', '--input-type', str(input_type)]):
        sim = SimStream('127.0.0.1', STATE_PORT, period_us=200)
        sim.enabled = True
        tx = Sender(ptype, bidir=bidir)
        try:
            time.sleep(2.2)                    # arm at zero throttle
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
            # motor must stop again at zero throttle
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
    with Sitl(sitl_path, ['--can-uri', CAN_URI, '--node-id', '10']):
        sim = SimStream('127.0.0.1', STATE_PORT, period_us=200)
        sim.enabled = True
        # some CI runners (github macos) have no multicast capable route
        # and the SITL cannot bring CAN up: skip rather than fail, with
        # the SITL log for diagnosis
        deadline = time.time() + 5
        while time.time() < deadline and not sim.samples:
            time.sleep(0.2)
        if not sim.samples:
            print('SKIP: SITL state stream never started with CAN enabled, '
                  'multicast is probably unavailable on this host. SITL log tail:')
            sys.stdout.flush()
            os.system('tail -5 sitl_ci.log')
            sim.close()
            return
        node = dronecan.make_node(CAN_URI, node_id=100, bitrate=1000000)
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


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    default_sitl = os.path.join(HERE, '..', '..', 'obj', 'AM32_AM32_SITL_CAN_2.20.elf')
    ap.add_argument('--sitl', default=os.path.normpath(default_sitl))
    args = ap.parse_args()

    if not os.path.exists(args.sitl):
        print('SITL binary not found: %s' % args.sitl)
        sys.exit(2)

    test_dshot(args.sitl, 'dshot600 bidir edt', sd.TYPE_DSHOT600,
               bidir=True, edt=True, value=800, rpm_lo=4000, rpm_hi=7000)
    test_dshot(args.sitl, 'dshot300', sd.TYPE_DSHOT300,
               bidir=False, edt=False, value=600, rpm_lo=3000, rpm_hi=6000)
    test_dshot(args.sitl, 'pwm', sd.TYPE_PWM,
               bidir=False, edt=False, value=1500, rpm_lo=4000, rpm_hi=9000,
               input_type=2)
    test_dronecan(args.sitl)

    if failures:
        print('\n%d FAILED: %s' % (len(failures), ', '.join(failures)))
        sys.exit(1)
    print('\nall tests passed')


if __name__ == '__main__':
    main()
