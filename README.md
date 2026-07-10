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

Optional static analysis / size helpers: `scripts/` and `make cppcheck` / related targets.

---

## Features (shared with upstream AM32)

- Firmware upgrade via Betaflight passthrough, single-wire serial, or related tools  
- Servo PWM and DShot (300 / 600), including bi-directional DShot  
- KISS-style ESC telemetry  
- Variable PWM frequency and sinusoidal startup for larger motors  
- Multi-vehicle use with a flight controller; crawler-oriented builds exist upstream  

Upstream feature docs and crawler notes: [AM32 wiki / crawler hardware](https://github.com/AlkaMotors/AM32-MultiRotor-ESC-firmware/wiki/Crawler-Hardware-and-AM32).

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
