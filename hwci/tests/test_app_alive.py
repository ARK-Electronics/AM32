"""_ensure_app_alive: bootloader-stuck ESC recovery on live-source bring-up.

On this rig the stand's inactive ESC output leaves the signal line high, and
the AM32 bootloader then never jumps to the app after a flash/power-cycle
(observed on the ARK 4IN1 bench: perf magic reads 0x00000000, PC loops in the
bootloader). The runner must drive zero throttle, reset, and wait for the
app's magic instead of handing a dead perf channel to the run.

A decodable magic alone is NOT liveness (both observed on the bench
2026-07-21):
- RAM persists across reset, so a stale perf struct still decodes while the
  ESC is parked in the bootloader -> loop_iters frozen.
- A half-booted app (vector-table force-boot) keeps the main loop spinning
  but its timer ISRs/ADC never start -> loop_iters advances, voltage == 0,
  and the ESC can never arm.
"""
from __future__ import annotations

import pytest

from hwci.perf import PerfDecodeError
from hwci.runner import _ensure_app_alive


class FakeSample:
    def __init__(self, loop_iters: int, voltage: float):
        self.loop_iters = loop_iters
        self.voltage = voltage


class FakeReader:
    """Dead in a configurable way until the fake target has been reset."""

    def __init__(self, alive_after_resets: int, dead_mode: str = "raise"):
        self.alive_after_resets = alive_after_resets
        self.dead_mode = dead_mode
        self.resets = 0
        self._iters = 0

    def read(self):
        if self.resets >= self.alive_after_resets:
            self._iters += 1
            return FakeSample(self._iters, 25.2)
        if self.dead_mode == "raise":
            raise PerfDecodeError("bad magic 0x00000000")
        if self.dead_mode == "frozen":
            return FakeSample(0, 0.0)  # stale RAM: decodes, never advances
        if self.dead_mode == "novolt":
            self._iters += 1
            return FakeSample(self._iters, 0.0)  # half-boot: ADC dead
        raise AssertionError(self.dead_mode)


class FakeDbg:
    def __init__(self, reader: FakeReader):
        self._reader = reader

    def reset_run(self):
        self._reader.resets += 1


class FakeThrottle:
    def __init__(self):
        self.commands = []

    def set(self, throttle):
        self.commands.append(throttle)

    def quiesce(self):
        self.commands.append("quiesce")


def test_already_alive_touches_nothing():
    reader = FakeReader(alive_after_resets=0)
    dbg, throttle = FakeDbg(reader), FakeThrottle()
    _ensure_app_alive(dbg, reader, throttle)
    assert reader.resets == 0
    assert throttle.commands == []


def test_stuck_in_bootloader_recovers_via_reset():
    reader = FakeReader(alive_after_resets=1)
    dbg, throttle = FakeDbg(reader), FakeThrottle()
    _ensure_app_alive(dbg, reader, throttle)
    assert reader.resets == 1
    # the signal must be DROPPED (not DShot-at-zero) before the reset so the
    # line is driven low and the bootloader jumps to the app
    assert throttle.commands == ["quiesce"]


def test_stale_magic_frozen_main_loop_recovers_via_reset():
    # Bootloader resident with the previous app run's perf struct still in
    # RAM: magic decodes but loop_iters never advances.
    reader = FakeReader(alive_after_resets=1, dead_mode="frozen")
    dbg, throttle = FakeDbg(reader), FakeThrottle()
    _ensure_app_alive(dbg, reader, throttle)
    assert reader.resets == 1
    assert throttle.commands == ["quiesce"]


def test_half_booted_app_zero_voltage_recovers_via_reset():
    # Main loop advancing but ADC/20 kHz path dead (voltage exactly 0):
    # the ESC would accept DShot yet never arm. Must be reset, not trusted.
    reader = FakeReader(alive_after_resets=1, dead_mode="novolt")
    dbg, throttle = FakeDbg(reader), FakeThrottle()
    _ensure_app_alive(dbg, reader, throttle)
    assert reader.resets == 1
    assert throttle.commands == ["quiesce"]


def test_never_alive_raises_actionable_error(monkeypatch):
    # collapse the wait loops so the failure path is fast
    import hwci.runner as runner
    monkeypatch.setattr(runner.time, "sleep", lambda s: None)
    clock = iter(range(0, 10_000))
    monkeypatch.setattr(runner.time, "monotonic", lambda: float(next(clock)))

    reader = FakeReader(alive_after_resets=99)
    dbg, throttle = FakeDbg(reader), FakeThrottle()
    with pytest.raises(RuntimeError, match="bootloader|HWCI_PERF"):
        _ensure_app_alive(dbg, reader, throttle)
    assert reader.resets == 2  # both attempts exhausted


def test_debugger_error_propagates():
    class DeadProbeReader:
        def read(self):
            raise ConnectionError("SWD gone")

    reader = DeadProbeReader()
    with pytest.raises(ConnectionError):
        _ensure_app_alive(FakeDbg(FakeReader(0)), reader, FakeThrottle())
