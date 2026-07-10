#!/usr/bin/env python3
'''
Shared helpers for AM32 SITL CI tests: process lifecycle, free UDP ports,
frame senders, RPM measurement from the state stream.
'''

from __future__ import annotations

import glob
import os
import socket
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
SITL_DIR = HERE
REPO_ROOT = os.path.normpath(os.path.join(HERE, '..', '..'))

sys.path.insert(0, HERE)
import sitl_dshot as sd  # noqa: E402
from sitl_gui_backend import SimStream  # noqa: E402


def find_sitl_binary(explicit=None):
    if explicit:
        return os.path.abspath(explicit)
    pat = os.path.join(REPO_ROOT, 'obj', 'AM32_AM32_SITL_CAN_*.elf')
    hits = sorted(glob.glob(pat))
    if not hits:
        return None
    return os.path.normpath(hits[-1])


def free_udp_port():
    '''bind a throwaway UDP socket to get an unused port number'''
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    return port


_mcast_seq = 0


def free_mcast_group():
    '''pick a mcast bus index 0..9 (SITL rejects anything outside that).

    Sequential tests get distinct buses so leftover traffic does not cross
    talk; wraps after 10.
    '''
    global _mcast_seq
    bus = _mcast_seq % 10
    _mcast_seq += 1
    return bus


class SitlStartError(RuntimeError):
    '''SITL process died during startup (e.g. mcast unavailable on CI).'''

    def __init__(self, returncode, log_tail):
        self.returncode = returncode
        self.log_tail = log_tail
        super().__init__('SITL exited early (code %s):\n%s' % (returncode, log_tail))

    @property
    def looks_like_mcast_failure(self):
        t = self.log_tail or ''
        # SIGPIPE (-13) during CAN init on macOS, or an explicit mcast error
        if self.returncode in (-13, 13):
            return True
        return ('CAN init mcast' in t and 'CAN on' not in t) or (
            'multicast' in t.lower() and 'failed' in t.lower())


class Sitl(object):
    '''start / stop one SITL instance with unique ports'''

    def __init__(self, sitl_path, extra_args=(), workdir=None, nosleep=True,
                 input_port=None, state_port=None, can_uri=None,
                 wait_s=0.6):
        self.workdir = workdir or os.getcwd()
        self.input_port = input_port if input_port is not None else free_udp_port()
        self.state_port = state_port if state_port is not None else free_udp_port()
        args = [sitl_path,
                '--input-port', str(self.input_port),
                '--state-port', str(self.state_port)]
        if can_uri is not None:
            args += ['--can-uri', can_uri]
        if nosleep:
            args.append('--nosleep')
        args += list(extra_args)
        self.log_path = os.path.join(self.workdir, 'sitl_ci.log')
        self.log = open(self.log_path, 'ab')
        self.proc = subprocess.Popen(
            args, stdout=self.log, stderr=self.log, cwd=self.workdir)
        time.sleep(wait_s)
        if self.proc.poll() is not None:
            self.log.flush()
            tail = ''
            try:
                with open(self.log_path, 'rb') as f:
                    tail = f.read()[-800:].decode(errors='replace')
            except OSError:
                pass
            raise SitlStartError(self.proc.returncode, tail)

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()

    def close(self):
        if self.proc.poll() is None:
            self.proc.kill()
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
        try:
            self.log.close()
        except Exception:
            pass
        # eeprom lock + file created in workdir
        for f in os.listdir(self.workdir):
            if f.startswith('am32_eeprom.bin'):
                try:
                    os.unlink(os.path.join(self.workdir, f))
                except OSError:
                    pass

    def log_tail(self, n=20):
        try:
            with open(self.log_path, 'rb') as f:
                lines = f.read().decode(errors='replace').splitlines()
            return '\n'.join(lines[-n:])
        except OSError:
            return ''


class Sender(object):
    '''background PWM/DShot frame sender with rate catch-up (CI-safe)'''

    def __init__(self, host, port, ptype, bidir=False, rate=500.0):
        self.port = sd.InputPort(host, port)
        self.ptype = ptype
        self.bidir = bidir
        self.rate = rate
        self.value = 1000 if ptype == sd.TYPE_PWM else 0
        self.cmds = []
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        try:
            self._run()
        except OSError:
            pass

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
        return -1.0
    return sum(s[1] for s in w) / len(w) * 60.0 / 6.28318


def wait_for_state(sim, timeout=5.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if sim.samples:
            return True
        time.sleep(0.1)
    return bool(sim.samples)


def open_state(host, port, period_us=200):
    sim = SimStream(host, port, period_us=period_us)
    sim.enabled = True
    return sim
