#!/usr/bin/env python3
'''
CI test for the SITL GUI: runs the real GUI under Qt's offscreen
platform, drives it through the control port (arming, EDT, throttle,
scopes, motor view) and asserts on the telemetry it reports.

usage: gui_ci_test.py --gui-python Mcu/SITL/venv/bin/python3
'''

from __future__ import annotations

import argparse
import os
import re
import socket
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sitl_harness import Sitl, find_sitl_binary, free_udp_port  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
failures = []


def check(name, cond, detail):
    status = 'PASS' if cond else 'FAIL'
    print('%s: %s (%s)' % (status, name, detail))
    sys.stdout.flush()
    if not cond:
        failures.append(name)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--gui-python', required=True)
    ap.add_argument('--sitl', default=None)
    args = ap.parse_args()

    sitl_path = find_sitl_binary(args.sitl)
    if not sitl_path or not os.path.exists(sitl_path):
        print('SITL binary not found: %s' % sitl_path)
        sys.exit(2)

    env = dict(os.environ)
    env['QT_QPA_PLATFORM'] = 'offscreen'
    control_port = free_udp_port()
    # free_udp_port returns a UDP port; TCP control port needs its own bind.
    # Re-bind via TCP to get a free TCP port.
    sprobe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sprobe.bind(('127.0.0.1', 0))
    control_port = sprobe.getsockname()[1]
    sprobe.close()

    # BDShot/EDT path only — keep CAN off so the GUI does not block on
    # mcast/SocketCAN bring-up (DroneCAN is covered by other tests).
    with Sitl(sitl_path, ['--input-type', '1'], can_uri='none') as sitl:
        gui = subprocess.Popen(
            [args.gui_python, os.path.join(HERE, 'sitl_gui.py'),
             '--control-port', str(control_port),
             '--port', str(sitl.input_port),
             '--state-port', str(sitl.state_port),
             '--can-uri', 'none'],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)

        # first launch can be slow (Qt font cache); retry the connection
        s = None
        deadline = time.time() + 45
        while time.time() < deadline:
            try:
                s = socket.create_connection(('127.0.0.1', control_port), timeout=5)
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
        # connect() timeout must not apply to the long idle gaps between
        # scripted commands (arm + EDT hold is several seconds).
        s.settimeout(None)
        f = s.makefile('r')
        responses = []

        def reader():
            try:
                for line in f:
                    responses.append(line.rstrip())
            except OSError:
                pass

        threading.Thread(target=reader, daemon=True).start()

        # Hold zero throttle long enough for bidir auto-detect, arming, and
        # EDT enable (firmware needs 6 identical DShot cmds while stopped).
        for delay, cmd in [
                (0.1, 'ds_type dshot600'), (0.1, 'ds_bidir 1'),
                (0.1, 'ds_enable 1'), (0.5, 'ds_edt 1'),
                (4.0, 'ds_value 900'),
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
