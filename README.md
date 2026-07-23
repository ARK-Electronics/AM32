# AM32 — ARK Electronics fork

Firmware for ARM-based brushless ESC (electronic speed controllers).

This repository is a **fork of [upstream AM32](https://github.com/am32-firmware/AM32)** maintained by **[ARK Electronics](https://arkelectron.com/)**. It tracks upstream capability while carrying ARK’s product line, refactor work, and test infrastructure.

| | Upstream | This fork |
|--|----------|-----------|
| Remote | [am32-firmware/AM32](https://github.com/am32-firmware/AM32) | [ARK-Electronics/AM32](https://github.com/ARK-Electronics/AM32) |
| Product branch | `main` | **`ark-release`** |
| Focus | Multi-vendor ESC firmware | ARK targets + maintainability + CI |

For stock AM32 releases, configurators, Discord, and community support, prefer **[am32.ca](https://am32.ca)** and the [upstream project](https://github.com/am32-firmware/AM32).

---

## What this fork adds

### Global refactor
Large control-path split out of a monolithic `main.c` into focused modules (runtime, settings, motor control helpers, and related MCU/F051 work). The goal is safer changes, clearer ownership of hot paths, and room for instrumentation without growing one file forever.

### SITL (software-in-the-loop)
Native Linux build of the firmware against a simulated motor / bridge / battery, with DroneCAN over multicast UDP. Useful for protocol, startup, and logic tests without ESC hardware.

- Overview: [Mcu/SITL/README.md](Mcu/SITL/README.md)
- CI: `.github/workflows/SITL.yml`

```bash
make arm_sdk_install   # once, for cross toolchain (SITL itself is host gcc)
make AM32_SITL_CAN
obj/AM32_AM32_SITL_CAN_*.elf --node-id 10 --verbose
```

### HITL / Hardware-CI
In-the-loop bench automation for the **ARK 4IN1** (and related F051 work): build/flash, drive the motor (Flight Stand and/or PX4 BDShot setups), read on-device performance counters over SWD, and produce metrics / pass-fail reports.

- Harness: [hwci/README.md](hwci/README.md)
- Bench setups: [hwci/docs/BENCH_SETUPS.md](hwci/docs/BENCH_SETUPS.md)
- CI: `.github/workflows/hwci.yml`

---

## Branch model

| Branch | Role |
|--------|------|
| **`ark-release`** | ARK integration line — open product PRs here |
| `main` | Mirrors / tracks upstream AM32 more closely |
| Feature branches | Short-lived; rebase onto `ark-release` unless targeting pure upstream work |

---

## Build (make + GCC)

IDE project trees (Keil / MRS) are **not** maintained in this fork. Use the Makefile and the pinned **xPack GNU Arm Embedded GCC** (see `make/tools.mk`).

```bash
# Install the pinned Arm toolchain into tools/<os>/
make arm_sdk_install

# List / build targets (examples)
make targets
make -j$(nproc) ARK_4IN1_F051
```

Firmware objects land under `obj/`. MCU families supported by the build system include F051, F031, G071, E230, F415, F421, L431, G431, V203, G031, A153, and SITL — exact product names live in `Inc/targets.h`.

Optional static analysis / size / format helpers:

```bash
make format            # apply clang-format (.clang-format) to app + MCU sources
make check_format      # fail if sources need formatting (used in PR CI)
make format_changed    # format only files changed vs origin/ark-release
make cppcheck          # static analysis of the ARK F051 control path
make size-check-ark    # ARK F051 + HWCI_PERF flash/RAM gate
```

Style is **PX4-inspired** via clang-format (Linux braces, tab indent width 8, `int *p`, column 140) — same `make format` / `check_format` workflow as PX4, not astyle itself. See `.clang-format`.

`make format` skips vendor trees (`Mcu/**/Drivers`, CMSIS, DroneCAN `dsdl_generated` / `libcanard`). Install clang-format with `pip install --user 'clang-format==22.1.5'` (version pinned to match CI) or your distro package.

---

## Features (shared with upstream AM32)

- Firmware upgrade via Betaflight passthrough, single-wire serial, or related tools  
- Servo PWM and DShot (300 / 600), including bi-directional DShot  
- KISS-style ESC telemetry  
- Variable PWM frequency and sinusoidal startup for larger motors  
- Multi-vehicle use with a flight controller; crawler-oriented builds exist upstream  

Upstream feature docs and crawler notes: [AM32 wiki / crawler hardware](https://github.com/AlkaMotors/AM32-MultiRotor-ESC-firmware/wiki/Crawler-Hardware-and-AM32).

---

## Motor beeps and sounds

AM32 has no speaker. Beeps are PWM on the motor phases so the windings act as a small transducer (same idea as other BLHeli-family ESCs). Implementation: [`Src/sounds.c`](Src/sounds.c) / [`Inc/sounds.h`](Inc/sounds.h). Volume is EEPROM `beep_volume` (0–11; DroneCAN param `BEEP_VOLUME`, default 5). Sounds only run when the motor is **not spinning** (idle / disarmed / zero throttle as applicable).

Pitch below is **relative** (higher PWM timer prescaler → lower pitch). Exact Hz depends on MCU clock and timer setup. The ARK signature tunes (startup and arm/beacon-4 morse) instead use fixed note frequencies via `playBJNote`.

### Quick reference

| When you hear it | Pattern (pitch) | Function | Meaning |
|------------------|-----------------|----------|---------|
| Power-up (brushless) | Morse **“ARK”** (·– / ·–· / –·–) rising C6 → E6 → G6 (or custom melody) | `playStartupTune` | Firmware booted and is ready for input |
| Power-up (brushed build) | 4 rising beeps | `playBrushedStartupTune` | Brushed-mode startup |
| Signal lost (after soft-reset) | Single short low blip on C5 (~70 ms) | `playSignalLostTone` | RC/input timeout; distinct from the ARK boot tune |
| Arm / throttle zero accepted | Morse **“R”** (·–·) on G6 | `playInputTune` | ESC armed / input lock-in (“roger”) |
| Arm + cell LVC enabled | That “R” **once per cell** | `playInputTune` × N | Detected pack cell count (`Vbat / 3.70`) |
| Stick cal entered (PWM) | Descending whoop/sweep | `playBeaconTune3` | Entered servo high/low calibration |
| Stick cal high done | 2 notes, **rising** | `playDefaultTone` | Max endpoint captured |
| Stick cal low done | 2 notes, **falling** | `playChangedTone` | Min endpoint saved to EEPROM |
| DShot beacon 1 or 5 | Same as “default” 2-note rising | `playDefaultTone` | Beacon / locate |
| DShot beacon 2 | 2 notes, **falling** | `playChangedTone` | Beacon |
| DShot beacon 3 | Descending sweep | `playBeaconTune3` | Beacon |
| DShot beacon 4 | Morse **“R”** (·–·) on E6 | `playInputTune2` | Beacon |
| DShot cmd 12 (save settings) | Rising if normal dir, falling if reversed | `playDefaultTone` / `playChangedTone` | Settings written |

### Startup

| Function | When | Pattern |
|----------|------|---------|
| **`playStartupTune`** | Normal brushless boot (after init; also CRSF path) | If the previous run soft-reset from an RC **signal timeout**, plays **`playSignalLostTone`** instead (see below). Else if EEPROM custom tune byte 0 is programmed (not `0xFF`): plays **BlueJay-compatible** melody from `eepromBuffer.tune[]` via `playBlueJayTune`. Otherwise default: the **ARK signature tune** — “ARK” in morse code (·– / ·–· / –·–), one letter per step up a C major arpeggio (C6 → E6 → G6), ~1.4 s total. |
| **`playSignalLostTone`** | Soft-reset after armed (~0.5 s) or disarmed (~2 s) input timeout (`faultPollSignalTimeout` → `NVIC_SystemReset`) | **One short low blip** on C5 (≈ 523 Hz, ~70 ms). Marked via a `.noinit` cookie before reset so cold boot still plays the full ARK tune. Linker `.noinit` is provided for **F051** and **G431** (gcc + Keil G431 scatter). |
| **`playBrushedStartupTune`** | `BRUSHED_MODE` builds only | **Four rising beeps** (~300 ms), phases 1–4 (prescalers 40 → 30 → 25 → 20). |
| **`playBlueJayTune`** | Custom startup only | Notes/rests encoded in EEPROM tune blob (configurator “custom startup music”). Inter-note pause can scale with tune header byte 3. |

Some AT32 F415 targets defer startup audio through `play_tone_flag` instead of calling the tune immediately at boot.

### Armed / input recognition

Played from the 20 kHz control path when the ESC transitions to armed-idle after a stable zero throttle (`Src/control_loop.c`).

| Function | When | Pattern |
|----------|------|---------|
| **`playInputTune`** | Armed with **low-voltage cutoff mode 1** (cell-based) **off**, or as each cell beep | **Morse “R”** (·–·, “roger — signal received”) on G6 (≈ 1568 Hz), ~320 ms (same busy-wait budget as the old tune). |
| **Cell-count beeps** | Armed **and** `low_voltage_cut_off == 1` | `cell_count = battery_voltage / 370` (≈ 3.70 V/cell), then **`playInputTune` once per cell** with ~100 ms gaps. Count the “R”s to read pack cell count. |
| **`playInputTune2`** | DShot beacon 4; also used as deferred arm beep on some AT415 builds | Same **morse “R”** one arpeggio step lower (E6, ≈ 1319 Hz) so the beacon is distinguishable from the arm tune. |

### Servo PWM stick calibration

Only for **servo PWM** input when stick calibration is not disabled. Sequence in `Src/signal.c`:

1. Hold **high stick** long enough → **`playBeaconTune3`** (descending multi-step whoop) — calibration mode entered.
2. Hold steady **max** until accepted → **`playDefaultTone`** (two notes, **rising**: lower then higher).
3. Move to **min** and hold until accepted → **`playChangedTone`** (two notes, **falling**: higher then lower) — endpoints saved.

### DShot special commands (beacons & save)

DShot commands run only when **armed**, **motor not running**, and the command is repeated enough times (`Src/dshot.c`). Beacons 1–5 set `play_tone_flag`; actual audio plays on the next idle/low-throttle slot in `setInput` (`Src/control_loop.c`).

| DShot cmd | `play_tone_flag` | Sound | Typical use |
|-----------|------------------|-------|-------------|
| **1** | 1 | `playDefaultTone` — 2 notes rising | Beacon 1 |
| **2** | 2 | `playChangedTone` — 2 notes falling | Beacon 2 |
| **3** | 3 | `playBeaconTune3` — long descending sweep | Beacon 3 |
| **4** | 4 | `playInputTune2` — morse “R” on E6 | Beacon 4 |
| **5** | 5 | `playDefaultTone` — same as beacon 1 | Beacon 5 |
| **12** | `1 + dir_reversed` | Rising if direction normal, falling if reversed | **Save settings** confirmation |

Other DShot commands (direction, bi-dir, EDT, programming mode, etc.) do **not** play a dedicated melody unless noted above. Direction set (7/8) currently has confirmation beeps commented out in source.

### Beacon sweep detail

**`playBeaconTune3`**: stepped descending pitch with phase stepping (~10 ms steps, prescaler from high down toward lower values). Used as DShot beacon 3 and as the “entered stick calibration” cue.

### Volume and silence

| Setting | Effect |
|---------|--------|
| **`beep_volume` 0–11** | Duty cycle of the beep PWM (`volume * 3` compare counts). 0 is effectively silent; higher is louder (still limited so the motor barely moves). |
| Motor spinning / throttle up | Deferred tone flags wait until throttle is at idle; beeps are not mixed into normal drive. |
| No throttle signal after boot | ESC may stay in bootloader or keep waiting for input — you may only hear the **startup** tune, not the arm tune, until a valid zero throttle is seen. |

### Defined but unused

| Function | Status |
|----------|--------|
| **`playDuskingTune`** | Implemented in `sounds.c` (ascending then peaking melody) but **not called** from current application code. Kept for compatibility / possible future use. |

### Source map

| File | Role |
|------|------|
| [`Src/sounds.c`](Src/sounds.c) | All melody generators |
| [`Src/main.c`](Src/main.c) | Startup tune at boot |
| [`Src/control_loop.c`](Src/control_loop.c) | Arm beeps, cell count, deferred DShot tones |
| [`Src/dshot.c`](Src/dshot.c) | DShot command → tone flag |
| [`Src/signal.c`](Src/signal.c) | Servo stick-calibration tones |
| [`Src/settings.c`](Src/settings.c) | Applies `beep_volume` from EEPROM |

---

## Configuration tools & stock firmware

These are **upstream / community** tools; they are not ARK-specific:

- [AM32 Configurator](https://am32.ca) (web) and [downloads](https://am32.ca/downloads)  
- [esc-configurator.com](https://esc-configurator.com/)  
- Bootloaders: [AM32-bootloader](https://github.com/am32-firmware/AM32-bootloader)  
- Target list: [`Inc/targets.h`](Inc/targets.h) (this tree) or [upstream targets.h](https://github.com/am32-firmware/AM32/blob/main/Inc/targets.h)

To put AM32 on a blank ESC you still need a matching MCU bootloader (ST-LINK / GD-LINK / CMSIS-DAP / AT-LINK, etc.), then flash application firmware with a configurator or one-wire serial path.

---

## Hardware (typical for this fork)

ARK work centers on **STM32F051** 4-in-1 ESCs and related F051 targets, while the tree still builds the broader AM32 MCU set above. Upstream also documents STSPIN32F0, G071, GD32E230, AT32F415/F421, and others — see their hardware notes and compatibility charts.

---

## Support

| Topic | Where |
|-------|--------|
| **ARK / this fork** | [ARK-Electronics/AM32](https://github.com/ARK-Electronics/AM32) issues and PRs on `ark-release` |
| **Upstream AM32** | [Discord](https://discord.gg/h7ddYMmEVV), [Patreon](https://www.patreon.com/user?u=44228479), [am32.ca](https://am32.ca) |

---

## License

GPL-3.0 — see [LICENSE](LICENSE). Same license family as upstream AM32.

---

## Upstream credits

AM32 exists because of its authors, sponsors, and community. This fork inherits that work; see the [upstream README](https://github.com/am32-firmware/AM32/blob/main/README.md) for the full sponsor and contributor lists.
