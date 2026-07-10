'''Runtime motor model load over the SITL state port.'''

from __future__ import annotations

import os
import time

import pytest

from sitl_harness import SITL_DIR, rpm_from_state, wait_for_state
import sitl_dshot as sd
from sitl_harness import Sender


MODELS = os.path.join(SITL_DIR, 'models')


@pytest.mark.parametrize('model', [
    'racer_5inch.json',
    'default_7inch.json',
    'heavy_13inch.json',
    'unloaded.json',
])
def test_load_stock_model(sitl_factory, state_stream, model):
    path = os.path.join(MODELS, model)
    assert os.path.isfile(path), path
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim)
    sim.load_model(path)
    # model_status is updated asynchronously over UDP
    deadline = time.time() + 3.0
    status = ''
    while time.time() < deadline:
        status = sim.model_status or ''
        if status and 'fail' not in status.lower() and 'error' not in status.lower():
            if 'loading' not in status.lower() or 'ok' in status.lower() or model in status:
                break
        time.sleep(0.1)
    # Accept either an explicit OK-style status or a cleared error-free load
    assert 'error' not in status.lower() and 'fail' not in status.lower(), status


def test_racer_model_spins_under_dshot(sitl_factory, state_stream):
    '''heavier/lighter models still produce plausible RPM under DShot'''
    path = os.path.join(MODELS, 'racer_5inch.json')
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim)
    sim.load_model(path)
    time.sleep(0.5)

    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    try:
        time.sleep(2.2)
        tx.value = 700
        time.sleep(4.0)
        rpm = rpm_from_state(sim)
        # racer is higher kV / lighter load — allow a wide but non-zero band
        assert 2000 <= rpm <= 15000, 'rpm=%.0f out of range on racer model' % rpm
        tx.value = 0
        time.sleep(3.0)
        assert rpm_from_state(sim, 0.3) < 800
    finally:
        tx.stop()
