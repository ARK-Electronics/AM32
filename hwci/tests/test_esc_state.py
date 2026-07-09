"""ESC state machine: edge table (from C) + behavioural host model tests."""
from __future__ import annotations

import pytest

from hwci.esc_state_model import (
    ESC_ALLOWED,
    ESC_ARMED_IDLE,
    ESC_ARMING,
    ESC_BRAKE,
    ESC_CLOSED_LOOP,
    ESC_DISARMED,
    ESC_FAULT_LVC,
    ESC_FAULT_SIGNAL,
    ESC_FAULT_STUCK,
    ESC_NAMES,
    ESC_OPEN_LOOP,
    ESC_SINE_START,
    ESC_STATE_COUNT,
    ESC_STUCK_LATCH,
    EscStateMachine,
    parse_esc_allowed_from_c,
    transition_allowed,
)


def test_edge_table_parsed_from_firmware_source():
    table = parse_esc_allowed_from_c()
    assert len(table) == ESC_STATE_COUNT
    assert table == ESC_ALLOWED
    # Sanity: DISARMED cannot jump to CLOSED_LOOP
    assert not transition_allowed(ESC_DISARMED, ESC_CLOSED_LOOP, table)


def test_self_transitions_always_allowed():
    for s in range(ESC_STATE_COUNT):
        assert transition_allowed(s, s), ESC_NAMES[s]


def test_happy_path_edges():
    path = [
        ESC_DISARMED, ESC_ARMING, ESC_ARMED_IDLE,
        ESC_OPEN_LOOP, ESC_CLOSED_LOOP,
    ]
    for a, b in zip(path, path[1:]):
        assert transition_allowed(a, b), f"{ESC_NAMES[a]} -> {ESC_NAMES[b]}"


def test_illegal_edges():
    assert not transition_allowed(ESC_DISARMED, ESC_OPEN_LOOP)
    assert not transition_allowed(ESC_ARMING, ESC_OPEN_LOOP)
    assert not transition_allowed(ESC_FAULT_LVC, ESC_DISARMED)
    assert not transition_allowed(ESC_FAULT_SIGNAL, ESC_ARMED_IDLE)
    assert not transition_allowed(ESC_CLOSED_LOOP, ESC_SINE_START)


@pytest.mark.parametrize("frm", range(ESC_STATE_COUNT))
def test_every_state_has_self_bit(frm: int):
    assert ESC_ALLOWED[frm] & (1 << frm)


# --- behavioural model -------------------------------------------------------


def test_arm_sequence_and_spin_up():
    m = EscStateMachine()
    m.inputSet = 1
    m.reconcile()
    assert m.state == ESC_ARMING

    m.to_armed_idle()
    assert m.state == ESC_ARMED_IDLE
    assert m.armed == 1 and m.running == 0

    m.old_routine = 1
    m.enter_running()
    assert m.state == ESC_OPEN_LOOP
    assert m.running == 1 and m.old_routine == 1
    assert m.may_six_step_throttle()
    assert m.in_poll_zc_drive()

    # ISR would clear old_routine after stable ZCs
    m.old_routine = 0
    m.reconcile()
    assert m.state == ESC_CLOSED_LOOP
    assert m.in_closed_loop()
    assert not m.in_poll_zc_drive()


def test_sine_start_handoff():
    m = EscStateMachine()
    m.to_armed_idle()
    m.to_sine_start()
    assert m.state == ESC_SINE_START
    assert m.in_sine_start()
    assert not m.may_six_step_throttle()

    m.sine_handoff_to_open_loop()
    assert m.state == ESC_OPEN_LOOP
    assert m.running == 1 and m.old_routine == 1
    assert m.prop_brake_active == 0


def test_stall_low_throttle_stops():
    m = EscStateMachine()
    m.to_closed_loop()
    m.input = 40
    m.note_stall_or_desync(stop_if_low_throttle=True)
    assert m.running == 0
    assert m.old_routine == 1
    assert m.state == ESC_ARMED_IDLE
    assert m.commutation_interval == 5000


def test_stall_high_throttle_falls_to_open_loop():
    m = EscStateMachine()
    m.to_closed_loop()
    m.input = 200
    m.note_stall_or_desync(stop_if_low_throttle=True)
    assert m.running == 1
    assert m.state == ESC_OPEN_LOOP
    assert m.old_routine == 1


def test_fault_stuck_cuts_input_and_latches():
    m = EscStateMachine()
    m.to_closed_loop()
    m.input = 1000
    m.to_fault_stuck()
    assert m.state == ESC_FAULT_STUCK
    assert m.input == 0
    assert m.running == 0
    assert m.bemf_timeout_happened == ESC_STUCK_LATCH
    assert m.is_fault()
    # reconcile keeps fault while latch present
    m.armed = 1
    m.running = 1
    m.reconcile()
    assert m.state == ESC_FAULT_STUCK


def test_fault_signal_clears_arming():
    m = EscStateMachine()
    m.to_armed_idle()
    m.inputSet = 1
    m.to_fault_signal()
    assert m.state == ESC_FAULT_SIGNAL
    assert m.armed == 0 and m.inputSet == 0
    m.reconcile()
    assert m.state == ESC_DISARMED  # no inputSet


def test_fault_lvc_is_sticky_via_reconcile():
    m = EscStateMachine()
    m.to_closed_loop()
    m.to_fault_lvc()
    assert m.state == ESC_FAULT_LVC
    assert m.LOW_VOLTAGE_CUTOFF == 1
    # named transition to DISARMED would be illegal; reconcile still forces LVC
    m.armed = 0
    m.inputSet = 0
    m.running = 0
    m.reconcile()
    assert m.state == ESC_FAULT_LVC


def test_illegal_named_edge_increments_counter_but_applies():
    m = EscStateMachine()
    # DISARMED -> OPEN_LOOP via to_open_loop is illegal
    assert m.state == ESC_DISARMED
    m.to_open_loop()
    assert m.illegal_edges == 1
    assert m.state == ESC_OPEN_LOOP  # still applied
    assert m.running == 1 and m.armed == 1


def test_brake_and_reconcile():
    m = EscStateMachine()
    m.to_armed_idle()
    m.to_brake()
    assert m.state == ESC_BRAKE
    assert m.in_brake()
    m.prop_brake_active = 0
    m.reconcile()
    assert m.state == ESC_ARMED_IDLE


def test_disarm_from_drive():
    m = EscStateMachine()
    m.to_closed_loop()
    m.to_disarmed()
    assert m.state == ESC_DISARMED
    assert m.armed == 0 and m.running == 0


def test_predicates_flag_backed_not_stale_state():
    """ISR may flip flags before reconcile; predicates follow flags."""
    m = EscStateMachine()
    m.to_closed_loop()
    assert m.state == ESC_CLOSED_LOOP
    # ISR sets poll path without reconcile yet
    m.old_routine = 1
    assert m.in_poll_zc_drive()
    assert m.in_open_loop()
    assert not m.in_closed_loop()
    # state enum still CLOSED until reconcile
    assert m.state == ESC_CLOSED_LOOP
    m.reconcile()
    assert m.state == ESC_OPEN_LOOP
