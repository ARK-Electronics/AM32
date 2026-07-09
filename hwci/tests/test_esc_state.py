"""Unit tests for the ESC drive state-machine transition table.

Mirrors ``Src/esc_state.c`` ``esc_allowed[]``. If a firmware edge change is
intentional, update both that array and ``ESC_ALLOWED`` here.
"""
from __future__ import annotations

import pytest

# Must match esc_state_t order in Inc/esc_state.h
ESC_DISARMED = 0
ESC_ARMING = 1
ESC_ARMED_IDLE = 2
ESC_SINE_START = 3
ESC_OPEN_LOOP = 4
ESC_CLOSED_LOOP = 5
ESC_BRAKE = 6
ESC_FAULT_STUCK = 7
ESC_FAULT_SIGNAL = 8
ESC_FAULT_LVC = 9
ESC_STATE_COUNT = 10

ESC_NAMES = [
    "DISARMED", "ARMING", "ARMED_IDLE", "SINE_START", "OPEN_LOOP",
    "CLOSED_LOOP", "BRAKE", "FAULT_STUCK", "FAULT_SIGNAL", "FAULT_LVC",
]

# Bitmasks identical to Src/esc_state.c esc_allowed[]
ESC_ALLOWED = [
    # DISARMED
    (1 << ESC_DISARMED) | (1 << ESC_ARMING) | (1 << ESC_ARMED_IDLE)
    | (1 << ESC_FAULT_SIGNAL) | (1 << ESC_FAULT_LVC),
    # ARMING
    (1 << ESC_ARMING) | (1 << ESC_DISARMED) | (1 << ESC_ARMED_IDLE)
    | (1 << ESC_FAULT_SIGNAL) | (1 << ESC_FAULT_LVC),
    # ARMED_IDLE
    (1 << ESC_ARMED_IDLE) | (1 << ESC_SINE_START) | (1 << ESC_OPEN_LOOP)
    | (1 << ESC_CLOSED_LOOP) | (1 << ESC_BRAKE) | (1 << ESC_DISARMED)
    | (1 << ESC_FAULT_STUCK) | (1 << ESC_FAULT_SIGNAL) | (1 << ESC_FAULT_LVC),
    # SINE_START
    (1 << ESC_SINE_START) | (1 << ESC_OPEN_LOOP) | (1 << ESC_ARMED_IDLE)
    | (1 << ESC_BRAKE) | (1 << ESC_DISARMED) | (1 << ESC_FAULT_STUCK)
    | (1 << ESC_FAULT_SIGNAL) | (1 << ESC_FAULT_LVC),
    # OPEN_LOOP
    (1 << ESC_OPEN_LOOP) | (1 << ESC_CLOSED_LOOP) | (1 << ESC_ARMED_IDLE)
    | (1 << ESC_BRAKE) | (1 << ESC_SINE_START) | (1 << ESC_DISARMED)
    | (1 << ESC_FAULT_STUCK) | (1 << ESC_FAULT_SIGNAL) | (1 << ESC_FAULT_LVC),
    # CLOSED_LOOP
    (1 << ESC_CLOSED_LOOP) | (1 << ESC_OPEN_LOOP) | (1 << ESC_ARMED_IDLE)
    | (1 << ESC_BRAKE) | (1 << ESC_DISARMED) | (1 << ESC_FAULT_STUCK)
    | (1 << ESC_FAULT_SIGNAL) | (1 << ESC_FAULT_LVC),
    # BRAKE
    (1 << ESC_BRAKE) | (1 << ESC_ARMED_IDLE) | (1 << ESC_OPEN_LOOP)
    | (1 << ESC_SINE_START) | (1 << ESC_DISARMED) | (1 << ESC_FAULT_STUCK)
    | (1 << ESC_FAULT_SIGNAL) | (1 << ESC_FAULT_LVC),
    # FAULT_STUCK
    (1 << ESC_FAULT_STUCK) | (1 << ESC_ARMED_IDLE) | (1 << ESC_DISARMED)
    | (1 << ESC_FAULT_SIGNAL) | (1 << ESC_FAULT_LVC),
    # FAULT_SIGNAL
    (1 << ESC_FAULT_SIGNAL) | (1 << ESC_DISARMED),
    # FAULT_LVC
    (1 << ESC_FAULT_LVC),
]


def transition_allowed(frm: int, to: int) -> bool:
    if not (0 <= frm < ESC_STATE_COUNT and 0 <= to < ESC_STATE_COUNT):
        return False
    if frm == to:
        return True
    return bool((ESC_ALLOWED[frm] >> to) & 1)


def test_allowed_table_length():
    assert len(ESC_ALLOWED) == ESC_STATE_COUNT
    assert len(ESC_NAMES) == ESC_STATE_COUNT


def test_self_transitions_always_allowed():
    for s in range(ESC_STATE_COUNT):
        assert transition_allowed(s, s), ESC_NAMES[s]


def test_happy_path_arm_to_closed_loop():
    path = [
        ESC_DISARMED, ESC_ARMING, ESC_ARMED_IDLE,
        ESC_OPEN_LOOP, ESC_CLOSED_LOOP,
    ]
    for a, b in zip(path, path[1:]):
        assert transition_allowed(a, b), f"{ESC_NAMES[a]} -> {ESC_NAMES[b]}"


def test_sine_handoff_and_stall():
    assert transition_allowed(ESC_ARMED_IDLE, ESC_SINE_START)
    assert transition_allowed(ESC_SINE_START, ESC_OPEN_LOOP)
    assert transition_allowed(ESC_CLOSED_LOOP, ESC_OPEN_LOOP)  # stall / desync
    assert transition_allowed(ESC_OPEN_LOOP, ESC_ARMED_IDLE)


def test_faults_from_drive():
    for drive in (ESC_OPEN_LOOP, ESC_CLOSED_LOOP, ESC_SINE_START, ESC_ARMED_IDLE):
        for fault in (ESC_FAULT_STUCK, ESC_FAULT_SIGNAL, ESC_FAULT_LVC):
            assert transition_allowed(drive, fault), (
                f"{ESC_NAMES[drive]} -> {ESC_NAMES[fault]}")


def test_illegal_edges():
    # Cannot free-run drive without arming first.
    assert not transition_allowed(ESC_DISARMED, ESC_OPEN_LOOP)
    assert not transition_allowed(ESC_DISARMED, ESC_CLOSED_LOOP)
    assert not transition_allowed(ESC_ARMING, ESC_OPEN_LOOP)
    # LVC is a dead end until power cycle (reconcile force only).
    assert not transition_allowed(ESC_FAULT_LVC, ESC_DISARMED)
    assert not transition_allowed(ESC_FAULT_LVC, ESC_ARMED_IDLE)
    # Signal fault only clears to disarmed (or self).
    assert not transition_allowed(ESC_FAULT_SIGNAL, ESC_ARMED_IDLE)
    assert not transition_allowed(ESC_FAULT_SIGNAL, ESC_OPEN_LOOP)
    # No reverse from closed loop into sine without going idle / open.
    assert not transition_allowed(ESC_CLOSED_LOOP, ESC_SINE_START)


def test_predicates_match_state_ranges():
    """Document predicate ranges used by firmware (escIsArmed / escIsDriving)."""
    armed = {ESC_ARMED_IDLE, ESC_SINE_START, ESC_OPEN_LOOP, ESC_CLOSED_LOOP, ESC_BRAKE}
    driving = {ESC_SINE_START, ESC_OPEN_LOOP, ESC_CLOSED_LOOP}
    faults = {ESC_FAULT_STUCK, ESC_FAULT_SIGNAL, ESC_FAULT_LVC}
    for s in range(ESC_STATE_COUNT):
        is_armed = s in armed
        is_driving = s in driving
        is_fault = s in faults
        assert is_fault == (s >= ESC_FAULT_STUCK)
        if is_driving:
            assert is_armed
        if is_fault:
            assert not is_armed and not is_driving


@pytest.mark.parametrize("frm", range(ESC_STATE_COUNT))
def test_every_state_has_at_least_self(frm: int):
    assert ESC_ALLOWED[frm] & (1 << frm)
