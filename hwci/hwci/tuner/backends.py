"""Tune backends: how a trial reaches an ESC (simulated or real)."""
from __future__ import annotations

import abc
import time
from pathlib import Path
from typing import Callable

from ..config import Profile, RigConfig
from ..debugger.base import MockDebugger
from ..model import RunResult
from ..runner import (build_live_sources, build_sim_sources, check_battery,
                      run_profile)
from ..settings import (DEFAULT_EEPROM_ADDRESS, EEPROM_SIZE, check_eeprom_layout,
                        default_blob, resolve_eeprom_address)


class TuneBackend(abc.ABC):
    """Writes settings pages and runs profiles against one ESC + rig."""

    eeprom_address: int
    mode: str                    # "sim" | "hw"

    @abc.abstractmethod
    def read_page(self) -> bytes:
        """Current 192-byte settings page on the device."""

    @abc.abstractmethod
    def program(self, blob: bytes, bin_path: Path) -> None:
        """Write ``blob`` as the settings page and reset into it."""

    @abc.abstractmethod
    def run_trial(self, blob: bytes, profile: Profile, bin_path: Path,
                  meta: dict, *, battery_cells: int | None,
                  min_cell_voltage: float) -> tuple[RunResult, dict]:
        """Program ``blob``, run ``profile``, return (result, extra) where
        extra carries ``settings_verified`` and ``resting_v``. Raises
        :class:`BatteryTooLowError` BEFORE arming if the pack is too low."""

    def wait_for_cool(self, max_temp_c: float, timeout_s: float) -> None:
        """Optionally block until the FET temp is below ``max_temp_c``."""

    def close(self) -> None:
        pass


class _SimEepromDevice(MockDebugger):
    """MockDebugger whose flash() actually lands: the blob becomes the
    readable page AND is decoded into the shared RigSimulator - the sim's
    equivalent of 'program page, reset, firmware loads settings at boot'."""

    def __init__(self, sim, eeprom_address: int):
        super().__init__(base=eeprom_address, size=EEPROM_SIZE)
        self._sim = sim

    def flash(self, bin_path: str, load_addr: int) -> None:
        from ..sim import SimSettings
        super().flash(bin_path, load_addr)
        blob = Path(bin_path).read_bytes()
        self.poke(load_addr, blob)
        self._sim.set_settings(SimSettings.from_blob(blob))


class SimTuneBackend(TuneBackend):
    """One persistent RigSimulator across all trials (pack state, temperature
    and RNG carry over, exactly like a physical rig would)."""

    mode = "sim"

    def __init__(self, rig: RigConfig | None = None, *,
                 motor_params=None, seed: int = 1234, noise: float = 0.01,
                 base_blob: bytes | None = None):
        from ..sim import MotorParams, RigSimulator
        self.rig = rig or RigConfig()
        params = motor_params or MotorParams(
            pole_pairs=self.rig.pole_pairs, demag_prone=True,
            # default sim tunes shouldn't randomly fail startup at the stock
            # startup_power of 100; tests inject a harsher fail_ref
            startup_fail_ref=100.0)
        self.sim = RigSimulator(params=params, seed=seed, noise=noise)
        self.eeprom_address = DEFAULT_EEPROM_ADDRESS
        self.dbg = _SimEepromDevice(self.sim, self.eeprom_address)
        self.dbg.poke(self.eeprom_address, base_blob or default_blob())
        # test hook: report a different resting pack voltage (e.g. fake a
        # drained pack to exercise the swap path)
        self.voltage_fn: Callable[[], float] | None = None

    def read_page(self) -> bytes:
        return self.dbg.read_memory(self.eeprom_address, EEPROM_SIZE)

    def program(self, blob: bytes, bin_path: Path) -> None:
        bin_path.parent.mkdir(parents=True, exist_ok=True)
        bin_path.write_bytes(blob)
        self.dbg.flash(str(bin_path), self.eeprom_address)

    def _resting_voltage(self) -> float:
        return self.voltage_fn() if self.voltage_fn else self.sim.voltage

    def run_trial(self, blob: bytes, profile: Profile, bin_path: Path,
                  meta: dict, *, battery_cells: int | None,
                  min_cell_voltage: float) -> tuple[RunResult, dict]:
        resting_v = self._resting_voltage()
        # Gate only on an injected voltage_fn (pack-drain tests): the sim's
        # nominal battery doesn't represent a real cell count, same reason
        # `hwci run`/`ci` ignore --battery-cells under --sim. A hardware
        # spec's battery_cells must not stop a sim dry-run of the same spec.
        if battery_cells and self.voltage_fn is not None:
            check_battery(resting_v, battery_cells, min_cell_voltage)
        self.program(blob, bin_path)
        verified = self.read_page() == blob
        sources = build_sim_sources(self.rig, profile, sim=self.sim)
        try:
            result = run_profile(profile, sources, realtime=False, meta=meta)
        finally:
            sources.close()
        return result, {"settings_verified": verified,
                        "resting_v": round(resting_v, 3)}

    def wait_for_cool(self, max_temp_c: float, timeout_s: float) -> None:
        waited = 0.0
        while self.sim.temp_c > max_temp_c and waited < timeout_s:
            self.sim.step(1.0, 0.0)   # cool at zero throttle, sim time
            waited += 1.0


class HwTuneBackend(TuneBackend):
    """Real rig: one-shot OpenOCD flash of the settings page, then the
    standard live sources (mirrors cmd_ci's flash-then-open flow)."""

    mode = "hw"

    def __init__(self, rig: RigConfig, *, tare: bool = True):
        from ..debugger.openocd import OpenOcdDebugger
        self.rig = rig
        self.tare = tare
        elf = rig.resolved_elf()
        if elf is None:
            raise FileNotFoundError(
                f"no ELF for target {rig.target} in {rig.resolved_obj_dir()}; "
                "build + flash firmware (HWCI_PERF=1) before tuning")
        self.elf_path = str(elf)
        # Layout drift between EEPROM_FIELDS and the flashed firmware is
        # caught HERE, before any page is written.
        check_eeprom_layout(self.elf_path)
        self._make_dbg = lambda: OpenOcdDebugger(
            rig.openocd_configs, openocd_bin=rig.openocd_bin,
            search_dirs=rig.openocd_search_dirs)
        dbg = self._make_dbg().open()
        try:
            self.eeprom_address = resolve_eeprom_address(dbg, self.elf_path)
        finally:
            dbg.close()

    def read_page(self) -> bytes:
        dbg = self._make_dbg().open()
        try:
            return dbg.read_memory(self.eeprom_address, EEPROM_SIZE)
        finally:
            dbg.close()

    def program(self, blob: bytes, bin_path: Path) -> None:
        bin_path.parent.mkdir(parents=True, exist_ok=True)
        bin_path.write_bytes(blob)
        # One-shot program of the 1KB settings sector + verify + reset; the
        # firmware reloads settings on the way back up.
        self._make_dbg().flash(str(bin_path), self.eeprom_address)

    def run_trial(self, blob: bytes, profile: Profile, bin_path: Path,
                  meta: dict, *, battery_cells: int | None,
                  min_cell_voltage: float) -> tuple[RunResult, dict]:
        from ..runner import _live_voltage
        self.program(blob, bin_path)
        sources = build_live_sources(
            self.rig, profile, battery_cells=battery_cells,
            min_cell_voltage=min_cell_voltage, tare=self.tare)
        try:
            verified = False
            if sources.perf_reader is not None:
                readback = sources.perf_reader.dbg.read_memory(
                    self.eeprom_address, EEPROM_SIZE)
                verified = readback == blob
            resting_v = _live_voltage(sources.stand, sources.perf_source)
            result = run_profile(profile, sources, realtime=True, meta=meta)
        finally:
            sources.close()
        return result, {
            "settings_verified": verified,
            "resting_v": round(resting_v, 3) if resting_v is not None else None}

    def wait_for_cool(self, max_temp_c: float, timeout_s: float) -> None:
        # Best-effort: poll the ESC's own temperature over a short live
        # session until it cools (the stand's FET probe needs open sources,
        # which would hold the throttle line - keep it simple).
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            page_dbg = self._make_dbg().open()
            try:
                from ..perf_reader import PerfReader
                reader = PerfReader(page_dbg, self.elf_path,
                                    check_layout=False)
                temp = reader.read().raw.get("temperature_c")
            except Exception:
                temp = None
            finally:
                page_dbg.close()
            if temp is None or temp <= max_temp_c:
                return
            time.sleep(10.0)
