'''Servo / PWM input path.'''

from __future__ import annotations

import time

import sitl_dshot as sd
from sitl_harness import Sender, rpm_from_state, wait_for_state


def test_pwm_midstick_spins_and_stops(sitl_factory, state_stream):
    sitl = sitl_factory(extra_args=['--input-type', '2'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim), sitl.log_tail()
    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_PWM)
    try:
        # arm at 1000 us
        tx.value = 1000
        time.sleep(2.2)
        tx.value = 1500
        time.sleep(4.0)
        rpm = rpm_from_state(sim)
        assert 4000 <= rpm <= 9000, 'rpm=%.0f expected 4000..9000' % rpm
        tx.value = 1000
        time.sleep(3.0)
        rpm = rpm_from_state(sim, 0.3)
        assert rpm < 500, 'did not stop: rpm=%.0f' % rpm
    finally:
        tx.stop()
