'''SITL process boot and basic I/O surface.'''

from __future__ import annotations

import os
import time

import pytest

from sitl_harness import wait_for_state


def test_boot_binds_ports_and_streams_state(sitl_factory, state_stream):
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim, timeout=5.0), (
        'state stream never started\n' + sitl.log_tail())
    latest = sim.latest()
    assert latest is not None
    # omega ~ 0 at idle, bus voltage from default model (~16.8 V)
    omega, vbus = latest[1], latest[10]
    assert abs(omega) < 50.0, 'motor spinning at idle: omega=%g' % omega
    assert 10.0 < vbus < 30.0, 'unexpected bus voltage: %g' % vbus


def test_boot_verbose_log_mentions_ports(sitl_factory):
    sitl = sitl_factory(extra_args=['--input-type', '1', '--verbose'],
                        can_uri='none', wait_s=1.0)
    # give the verbose 1 Hz line a chance; port banners print at startup
    time.sleep(0.5)
    log = sitl.log_tail(40)
    assert 'PWM/DShot input on udp port' in log or str(sitl.input_port) in log, log
    assert 'state/model port' in log or str(sitl.state_port) in log, log


def test_eeprom_file_created(sitl_factory, workdir):
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none', wait_s=0.8)
    eeprom = os.path.join(workdir, 'am32_eeprom.bin')
    assert os.path.exists(eeprom), 'eeprom not created in %s: %s' % (
        workdir, os.listdir(workdir))
    assert os.path.getsize(eeprom) > 0
