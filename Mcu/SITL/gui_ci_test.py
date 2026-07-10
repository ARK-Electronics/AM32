#!/usr/bin/env python3
'''
CI test for the SITL GUI: runs the real GUI under Qt's offscreen
platform, drives it through the control port (arming, EDT, throttle,
scopes, motor view) and asserts on the telemetry it reports.

usage: gui_ci_test.py --gui-python Mcu/SITL/venv/bin/python3
'''

import argparse
import glob
import os
import re
import socket
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from run_ci_tests import Sitl, check, failures, INPUT_PORT, STATE_PORT

HERE = os.path.dirname(os.path.abspath(__file__))
CONTROL_PORT = 57899


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--gui-python', required=True)
    # don't hardcode the firmware version in the default binary path
    pat = os.path.join(HERE, '..', '..', 'obj', 'AM32_AM32_SITL_CAN_*.elf')
    hits = sorted(glob.glob(pat))
    ap.add_argument('--sitl', default=os.path.normpath(hits[0]) if hits else None)
    args = ap.parse_args()

    env = dict(os.environ)
    env['QT_QPA_PLATFORM'] = 'offscreen'

    with Sitl(args.sitl, ['--can-uri', 'none', '--input-type', '1']):
        gui = subprocess.Popen(
            [args.gui_python, os.path.join(HERE, 'sitl_gui.py'),
             '--control-port', str(CONTROL_PORT),
             '--port', str(INPUT_PORT), '--state-port', str(STATE_PORT),
             '--can-uri', 'mcast:7'],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)

        # first launch can be slow (Qt font cache); retry the connection
        responses = []
        s = None
        deadline = time.time() + 45
        while time.time() < deadline:
            try:
                s = socket.create_connection(('127.0.0.1', CONTROL_PORT), timeout=5)
                break
            except OSError:
                if gui.poll() is not None:
                    print(gui.stdout.read() if gui.stdout else '')
                    print('gui exited before control port came up')
                    sys.exit(1)
                time.sleep(1.0)
        if s is None:
            gui.kill()
            print('control port never came up')
            sys.exit(1)
        f = s.makefile('r')

        def reader():
            for line in f:
                responses.append(line.rstrip())

        threading.Thread(target=reader, daemon=True).start()

        for delay, cmd in [
                (0.1, 'ds_type dshot600'), (0.1, 'ds_bidir 1'),
                (0.1, 'ds_enable 1'), (0.3, 'ds_edt 1'),
                (2.5, 'ds_value 900'),
                (1.0, 'graph_i 1'), (0.2, 'graph_v 1'), (0.2, 'motorview 1'),
                (4.0, 'status'),
                (1.0, 'quit')]:
            time.sleep(delay)
            s.sendall((cmd + '\n').encode())

        try:
            gui.wait(timeout=20)
            check('gui exits cleanly', gui.returncode == 0,
                  'exit=%s' % gui.returncode)
        except subprocess.TimeoutExpired:
            gui.kill()
            check('gui exits cleanly', False, 'hung')

        bds = [r for r in responses if r.startswith('STATUS BDShot')]
        check('gui got BDShot status', len(bds) > 0, '%d lines' % len(bds))
        if bds:
            m = re.search(r'rpm=(\d+)\s+(\w+)\s+EDT:(\w+)', bds[-1])
            check('gui status parses', m is not None, bds[-1])
            if m:
                rpm, spin, edt = int(m.group(1)), m.group(2), m.group(3)
                check('gui rpm', 4000 <= rpm <= 7000, 'rpm=%d' % rpm)
                check('gui spinning', spin == 'spinning', spin)
                check('gui edt on', edt == 'on', 'EDT:%s' % edt)
        out = gui.stdout.read() if gui.stdout else ''
        check('gui no tracebacks', 'Traceback' not in out,
              (out[-300:] if 'Traceback' in out else 'clean'))

    if failures:
        print('\n%d FAILED' % len(failures))
        sys.exit(1)
    print('\ngui test passed')


if __name__ == '__main__':
    main()
