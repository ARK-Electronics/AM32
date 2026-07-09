"""Host-side model of Src/esc_state.c for unit tests.

The edge table is **parsed from the firmware source** so Python cannot drift
from C. Transition and reconcile behaviour mirrors esc_state.c closely enough
to lock policy contracts without cross-compiling the ESC firmware.
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path

# --- enum (must match Inc/esc_state.h order) ---------------------------------
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

ESC_STUCK_LATCH = 102

# C identifiers in esc_state.h / esc_allowed[]
_ENUM = {f"ESC_{name}": i for i, name in enumerate(ESC_NAMES)}
_ENUM["ESC_STATE_COUNT"] = ESC_STATE_COUNT


def _repo_esc_state_c() -> Path:
    # hwci/hwci/esc_state_model.py -> repo root
    return Path(__file__).resolve().parents[2] / "Src" / "esc_state.c"


def parse_esc_allowed_from_c(path: Path | None = None) -> list[int]:
    """Parse ``esc_allowed[]`` designated initializers from esc_state.c."""
    src = (path or _repo_esc_state_c()).read_text()
    m = re.search(
        r"static const uint16_t esc_allowed\[ESC_STATE_COUNT\]\s*=\s*\{(.*?)\};",
        src,
        re.S,
    )
    if not m:
        raise ValueError("esc_allowed[] not found in esc_state.c")
    body = m.group(1)
    # [ESC_FOO] = expr  (expr ends at the trailing comma or closing brace)
    entries = re.findall(
        r"\[(ESC_[A-Z_]+)\]\s*=\s*(.*?)(?=\s*,\s*(?:/\*|\[)|\s*,\s*$|\s*\})",
        body,
        re.S | re.M,
    )
    # Drop comment-only noise; require ESC_* keys only
    entries = [(n, e) for n, e in entries if n.startswith("ESC_")]
    if len(entries) != ESC_STATE_COUNT:
        raise ValueError(
            f"expected {ESC_STATE_COUNT} esc_allowed entries, got {len(entries)}: "
            f"{[n for n, _ in entries]}")

    def eval_expr(expr: str) -> int:
        e = expr.strip().rstrip(",")
        e = re.sub(r"/\*.*?\*/", "", e)
        e = re.sub(r"\s+", " ", e)
        # Drop C integer suffixes (1u, 1U) without stripping letters from names.
        e = re.sub(r"(\d+)[uU]\b", r"\1", e)

        def repl_shift(mo: re.Match) -> str:
            name = mo.group(1)
            if name not in _ENUM:
                raise ValueError(f"unknown state in mask: {name}")
            return str(1 << _ENUM[name])

        e = re.sub(r"\(1\s*<<\s*(ESC_[A-Z_]+)\)", repl_shift, e)
        # Only allow digits, |, spaces, parens after substitution
        if re.search(r"[^0-9|() ]", e):
            raise ValueError(f"unsafe mask expression: {expr!r} -> {e!r}")
        return int(eval(e))  # noqa: S307 — constrained expression

    out = [0] * ESC_STATE_COUNT
    for name, expr in entries:
        if name not in _ENUM or name == "ESC_STATE_COUNT":
            raise ValueError(f"bad designated init key {name}")
        out[_ENUM[name]] = eval_expr(expr)
    return out


# Lazy-loaded from firmware; tests call parse at import for fail-fast.
ESC_ALLOWED: list[int] = parse_esc_allowed_from_c()


def transition_allowed(frm: int, to: int, table: list[int] | None = None) -> bool:
    table = table if table is not None else ESC_ALLOWED
    if not (0 <= frm < ESC_STATE_COUNT and 0 <= to < ESC_STATE_COUNT):
        return False
    if frm == to:
        return True
    return bool((table[frm] >> to) & 1)


@dataclass
class EscStateMachine:
    """Behavioural twin of esc_state.c (flags + named transitions + reconcile)."""

    state: int = ESC_DISARMED
    illegal_edges: int = 0

    armed: int = 0
    running: int = 0
    stepper_sine: int = 0
    old_routine: int = 1
    prop_brake_active: int = 0
    inputSet: int = 0
    input: int = 0
    bemf_timeout_happened: int = 0
    LOW_VOLTAGE_CUTOFF: int = 0
    commutation_interval: int = 12500

    def transition_allowed(self, frm: int, to: int) -> bool:
        return transition_allowed(frm, to)

    def _commit(self, next_state: int) -> None:
        if self.state != next_state and not self.transition_allowed(self.state, next_state):
            if self.illegal_edges < 0xFFFF:
                self.illegal_edges += 1
        self.state = next_state

    def _force(self, next_state: int) -> None:
        self.state = next_state

    # --- predicates (flag-backed, as in firmware) ---------------------------

    def is_fault(self) -> bool:
        return ESC_FAULT_STUCK <= self.state < ESC_STATE_COUNT

    def is_armed(self) -> bool:
        return self.armed != 0

    def is_driving(self) -> bool:
        return self.running != 0 or self.stepper_sine != 0

    def in_sine_start(self) -> bool:
        return self.stepper_sine != 0

    def in_open_loop(self) -> bool:
        return self.running != 0 and self.old_routine != 0

    def in_closed_loop(self) -> bool:
        return self.running != 0 and self.old_routine == 0 and not self.stepper_sine

    def in_brake(self) -> bool:
        return self.prop_brake_active != 0 and self.running == 0

    def may_six_step_throttle(self) -> bool:
        return self.armed != 0 and self.stepper_sine == 0

    def in_poll_zc_drive(self) -> bool:
        return self.old_routine != 0 and self.running != 0

    # --- reconcile ---------------------------------------------------------

    def reconcile(self) -> None:
        if self.bemf_timeout_happened == ESC_STUCK_LATCH:
            self._force(ESC_FAULT_STUCK)
            return
        if self.LOW_VOLTAGE_CUTOFF:
            self._force(ESC_FAULT_LVC)
            return
        if not self.armed:
            self._force(ESC_ARMING if self.inputSet else ESC_DISARMED)
            return
        if self.stepper_sine:
            self._force(ESC_SINE_START)
            return
        if self.prop_brake_active and not self.running:
            self._force(ESC_BRAKE)
            return
        if self.running:
            self._force(ESC_OPEN_LOOP if self.old_routine else ESC_CLOSED_LOOP)
            return
        self._force(ESC_ARMED_IDLE)

    # --- named transitions -------------------------------------------------

    def to_disarmed(self) -> None:
        self.armed = 0
        self.running = 0
        self.stepper_sine = 0
        self._commit(ESC_DISARMED)

    def to_arming(self) -> None:
        self.armed = 0
        self._commit(ESC_ARMING)

    def to_armed_idle(self) -> None:
        self.armed = 1
        self.running = 0
        self.stepper_sine = 0
        self._commit(ESC_ARMED_IDLE)

    def to_sine_start(self) -> None:
        self.armed = 1
        self.stepper_sine = 1
        self._commit(ESC_SINE_START)

    def to_open_loop(self) -> None:
        self.armed = 1
        self.running = 1
        self.old_routine = 1
        self.stepper_sine = 0
        self._commit(ESC_OPEN_LOOP)

    def to_closed_loop(self) -> None:
        self.armed = 1
        self.running = 1
        self.old_routine = 0
        self.stepper_sine = 0
        self._commit(ESC_CLOSED_LOOP)

    def to_brake(self) -> None:
        self.armed = 1
        self.prop_brake_active = 1
        self._commit(ESC_BRAKE)

    def to_fault_stuck(self) -> None:
        self.input = 0
        self.bemf_timeout_happened = ESC_STUCK_LATCH
        self.running = 0
        self.stepper_sine = 0
        self._commit(ESC_FAULT_STUCK)

    def to_fault_signal(self) -> None:
        self.armed = 0
        self.input = 0
        self.inputSet = 0
        self.running = 0
        self.stepper_sine = 0
        self._commit(ESC_FAULT_SIGNAL)

    def to_fault_lvc(self) -> None:
        self.LOW_VOLTAGE_CUTOFF = 1
        self.input = 0
        self.running = 0
        self.armed = 0
        self.stepper_sine = 0
        self._commit(ESC_FAULT_LVC)

    def enter_running(self) -> None:
        self.armed = 1
        self.running = 1
        self.stepper_sine = 0
        if not self.old_routine:
            self._commit(ESC_CLOSED_LOOP)
        else:
            self._commit(ESC_OPEN_LOOP)

    def sine_handoff_to_open_loop(self) -> None:
        self.stepper_sine = 0
        self.running = 1
        self.old_routine = 1
        self.prop_brake_active = 0
        self._commit(ESC_OPEN_LOOP)

    def note_stall_or_desync(self, stop_if_low_throttle: bool) -> None:
        self.old_routine = 1
        if stop_if_low_throttle and self.input < 48:
            self.running = 0
            self.commutation_interval = 5000
            self._commit(ESC_ARMED_IDLE if self.armed else ESC_DISARMED)
        elif self.running:
            self._commit(ESC_OPEN_LOOP)
        elif self.armed:
            self._commit(ESC_ARMED_IDLE)
        else:
            self._commit(ESC_DISARMED)
