# SETUP B — ARK FPV BDShot host

> **This document is only for SETUP B.** ESC signal is driven by ARK FPV / PX4
> (BDShot), not by the Flight Stand. For Flight Stand throttle (SETUP A), see
> [BENCH_SETUPS.md](BENCH_SETUPS.md) and `config/rig.flightstand.yaml`.
> Do not connect both hosts to the signal pin at once.

This branch instruments AM32 so a **BDShot host** (ARK FPV + PX4) can be
correlated against SWD firmware counters and, optionally, Flight Stand RPM.

## What was added (HWCI_PERF BDShot block)

On `main_instrumented` this is **struct v3** (appended after v2 zc jitter).
On `feat/split-main-control` the same fields are **struct v6** (after v5
esc_state). Host decoder keeps all historical versions.

Firmware `hwci_perf` (build with `HWCI_PERF=1`) appends:

| Field | Meaning |
|-------|---------|
| `dshot_rx_good` | Monotonic good-CRC DShot frames |
| `dshot_rx_bad` | Monotonic bad-CRC frames |
| `dshot_tx_frames` | BDShot reply packages built (`make_dshot_package`) |
| `dshot_last_com_us` | Period last packed into a reply (`e_com_time` units) |
| `dshot_telem_mode` | `0` = uni DShot, `1` = BDShot latched (idle-high detect) |
| `dshot_edt_mode` | Extended DShot Telemetry enable |

Host decoder: `hwci/hwci/perf.py` (v1/v2/v3). CSV columns: `perf_dshot_*`.

**Note:** `eepromBuffer.bi_direction` is *motor reverse / 3D mode*, not BDShot.
BDShot is the `dshot_telemetry` path (idle-high line, inverted CRC, GCR reply).

## Layers

```
  ARK FPV (PX4 1.17) USB CDC ── MAVLink ──► host capture script / QGC
       │
       ├─ BDShot+EDT signal ──► ARK 4IN1 AM32  ──► GCR eRPM / EDT frames
       ├─ serial telem UART  ──► KISS V/A/°C   ──► esc_status (parallel path)
       └─ optional MAVLink shell (NSH) for dshot / listener

  Optional ST-Link SWD on ESC ──► hwci_perf v3 (dshot_telem_mode, rx/tx, e_rpm)
```

## Recommended bench setup (USB ARK FPV)

What you described is enough for a **host-side BDShot baseline** without the
Flight Stand in the signal path:

| Link | Role |
|------|------|
| USB (ACM) | MAVLink: `esc_status`, logs, shell |
| Motor out → ESC signal | BDShot300/600 + EDT replies |
| ESC telem pad → FC UART | Classic KISS serial telem (V/A/temp/eRPM) |
| ST-Link on ESC SWD | Optional; proves ESC latched BDShot (`dshot_telem_mode`) |
| NSH / debug shell | Very useful — see below |

### PX4 config checklist (1.17)

1. **Actuators** → motor protocol **BDShot300** or **BDShot600** (not plain DShot).
2. Motor **pole count** matches the bench motor (e.g. 14 poles → 7 pairs).
3. **EDT**: enabled with BDShot on builds that support it (1.16+); confirms as
   non-zero temp/voltage/current in `esc_status` once spinning.
4. **Serial telem**: assign the UART in parameters / Actuators telemetry pin;
   baud 115200 is typical for AM32 KISS. This is a *second* path — useful to
   cross-check EDT vs wire telem, not a substitute for BDShot RPM rate.
5. Disarm safety / prop removed (or no-prop free-run) before motor test.

### Is the MAVLink / NSH shell helpful?

**Yes — use it.** Over USB you usually get both:

- **MAVLink** on the CDC ACM (QGC, mavproxy, capture script)
- **NSH** via `mavlink shell` (or a second ACM / UART if you wire the
  console)

Useful shell commands while a motor test runs:

```text
listener esc_status
listener esc_status -n 20          # a few samples
dshot status                       # if the dshot module exposes it
dshot esc_info -m 1                # may need telem; AM32 wants cmd ×6
work_queue status
```

Motor spin without QGC (examples vary by board; QGC Motor Test is safer):

```text
# Prefer QGC Actuators → Motor Test for first bring-up.
# Shell actuator tests differ by PX4 version — if unsure, use QGC.
```

Wire the **console UART** only if USB MAVLink shell is flaky; for this work
USB MAVLink + `mavlink shell` is usually enough.

### Capture on the PC

```bash
# Identify which ACM is PX4 (plug FPV USB, unplug other CDC gadgets if confused)
cd hwci
./scripts/px4_bdshot_capture.py --port /dev/ttyACM0 --discover 3

# Log while you motor-test from QGC (prop off!)
./scripts/px4_bdshot_capture.py --port /dev/ttyACM0 --duration 40 \
  -o runs/bdshot_px4_$(date +%Y%m%d_%H%M%S).csv
```

While logging, step throttle 0 → 20% → 50% → 80% → 0 in Motor Test.

Pass/fail (host-only):

| Check | Pass |
|-------|------|
| HEARTBEAT on ACM | link up |
| `ESC_STATUS` while spinning | BDShot and/or serial telem alive |
| RPM rises with throttle | decode + poles plausible |
| RPM → 0 at zero throttle | no stuck feedback |
| voltage/current/temp non-zero | EDT and/or serial telem path |

If RPM works but V/A/temp stay zero: BDShot eRPM OK, EDT/serial not configured.
If nothing in `ESC_STATUS`: wrong protocol (plain DShot), wrong motor index, or
not spinning.

## Phase A — flash instrumented ESC (optional SWD truth)

1. Flash instrumented image:
   ```bash
   make ARK_4IN1_F051 HWCI_PERF=1
   # flash channel under test via OpenOCD / hwci flash
   ```
2. With PX4 driving BDShot, SWD should show:
   - `dshot_telem_mode == 1`
   - `dshot_edt_mode == 1` after EDT enable
   - `dshot_rx_good` / `dshot_tx_frames` climbing
   - `dshot_rx_bad` near 0
3. Confirm on PX4:
   ```text
   listener esc_status
   ```
   Expect non-zero RPM when spinning; zeros at disarmed/zero throttle.
4. Optional: `logger on` → step throttle → `logger off` → pull ulog.

## Phase B — three-way correlation (SWD + PX4)

With the ESC still on the HWCI SWD probe:

```bash
# Terminal 1: poll SWD while PX4 drives the signal wire
cd hwci && .venv/bin/python - <<'PY'
import time
from hwci.config import load_rig
from hwci.debugger.openocd import OpenOCDDebugger
from hwci.perf_reader import PerfReader

rig = load_rig("rig.yaml")
dbg = OpenOCDDebugger(...)  # same openocd configs as hwci flash
# Or use: hwci run with a long idle profile while PX4 is the real throttle
# source — see Phase C when a px4 throttle backend exists.
PY
```

Practical interim approach:

1. Disconnect Flight Stand ESC output from the signal pin.
2. Connect **PX4 motor output** → ESC signal (common GND).
3. Keep ST-Link on channel-1 SWD.
4. Drive motor from PX4; watch live:

   ```bash
   cd hwci
   .venv/bin/python -m hwci flash --config rig.yaml --bin ../obj/AM32_ARK_4IN1_F051_*.bin
   # Use OpenOCD live mem, or a short custom poller reading hwci_perf
   ```

Expected SWD under true BDShot:

| Field | Expectation |
|-------|-------------|
| `dshot_telem_mode` | **1** after arm (idle-high auto-detect) |
| `dshot_rx_good` | Rate ≈ PX4 DShot rate (e.g. ~1 kHz) |
| `dshot_rx_bad` | Near 0; rising bad ⇒ line/CRC/load issue |
| `dshot_tx_frames` | Tracks RX when armed (reply each frame) |
| `e_rpm` | Matches PX4 `esc_status` / stand RPM within poles |

If `dshot_telem_mode` stays **0**, the host is still uni-directional DShot
(or the line is not idle-high between frames).

## Phase C — automated profile (later)

- Profile stub: `bdshot_smoke` (throttle steps for free-run).
- Still needs a `throttle_backend: px4` (MAVLink motor test / actuator) to
  fully automate; until then use Phase A/B with this firmware baseline.
- SITL already covers protocol decode: `Mcu/SITL/tests/test_dshot.py`
  (`test_dshot600_bidir_edt`).

## Size impact (F051 + HWCI_PERF)

v2 → v3 adds **16 bytes RAM** for the new fields (struct 80 → 96). Flash
delta is a few increments in `dshot.c` only when `HWCI_PERF=1`.

## Bench PSU safety (abrupt stop)

An **abrupt motor stop** (ACTUATOR_TEST timeout expiry, hard cmd→0, or
high-rate command thrash that kills spin) can back-feed / spike the 3 A
bench supply and put it into **fault mode**. Then subsequent runs show
`servo_raw` changing but **RPM=0** until the supply is reset.

Rules for PX4 motor tests on this bench:

1. Re-fire `ACTUATOR_TEST` every ~2 s (`timeout=3`) so the ~3 s cap never
   hard-cuts mid-hold.
2. Do **not** stream COMMAND_LONG at 10–50 Hz.
3. **Ramp down** over several seconds at end of run; never jump high→0.
4. If the supply faults, reset it before the next spin test.

Script: `hwci/scripts/px4_motor_stream.py` (refresh + slew + ramp-down).
