# Two exclusive bench setups

This harness supports **two different physical setups**. Use **one at a time** —
only one device may drive the ESC signal wire.

| | **SETUP A — Flight Stand** | **SETUP B — ARK FPV BDShot** |
|--|----------------------------|------------------------------|
| **Who drives ESC signal** | Flight Stand ESC output (uni DShot/PWM) | ARK FPV motor out (BDShot300/600 + EDT) |
| **PX4** | Not in the path | Required (USB MAVLink) |
| **BDShot replies** | No (stand is not a BDShot master) | Yes (eRPM / EDT on signal wire) |
| **Thrust / torque / optical RPM** | Flight Stand gRPC | Optional (stand sensors only; pin disconnected from stand ESC out) |
| **SWD `hwci_perf`** | Yes (ST-Link) | Optional (same ST-Link) |
| **Primary rig file** | `rig.yaml` or `config/rig.flightstand.yaml` | `config/rig.px4_bdshot.yaml` |
| **How to command motor** | `hwci run --profile noprop_…` | `scripts/px4_motor_stream.py` (+ capture) |
| **Typical profiles** | `noprop_smoke*`, `efficiency_sweep`, demag, … | `bdshot_smoke` (SWD log only; throttle is PX4) |

## SETUP A — Flight Stand throttle (no PX4 / no BDShot)

```
  Linux host ──gRPC──► Flight Stand ──ESC signal (uni DShot)──► AM32
       │                    │
       └──SWD (ST-Link)─────┴── optional optical RPM / thrust / current
```

- Wire ESC **signal** to the Flight Stand ESC output only.
- Do **not** connect ARK FPV motor outputs to that pin at the same time.
- Config: see `config/rig.flightstand.yaml` (active bench file is usually `rig.yaml`).
- Examples:

```bash
cd hwci
# flash + free-run smoke (stand drives throttle)
.venv/bin/python -m hwci flash --config rig.yaml --bin ../obj/AM32_ARK_4IN1_F051_*.bin
.venv/bin/python -m hwci run --config rig.yaml --profile noprop_smoke_100pct_3a --out runs/stand-smoke-1
```

## SETUP B — ARK FPV BDShot (no Flight Stand throttle)

```
  Linux host ──USB──► ARK FPV (PX4) ──BDShot+EDT signal──► AM32
       │                  │
       │                  └── optional KISS serial telem UART
       └──SWD (ST-Link) optional hwci_perf dshot_* counters
```

- Wire ESC **signal** to the FPV motor pad only (not stand ESC out).
- Flight Stand may stay powered for **sensors** only if the signal pin is free;
  typically leave stand throttle disconnected for this setup.
- Config: `config/rig.px4_bdshot.yaml` (`throttle_backend: none`).
- Motor command + eRPM live on PX4; scripts:

```bash
cd hwci
# flash instrumented FW (throttle not driven by harness)
.venv/bin/python -m hwci flash --config config/rig.px4_bdshot.yaml \
  --bin ../obj/AM32_ARK_4IN1_F051_*.bin

# drive motor via PX4 ACTUATOR_TEST (soft re-fire + ramp-down)
./scripts/px4_motor_stream.py --port /dev/ttyACM2 \
  --steps 0.12:5,0.25:5,0.40:6 --refresh 1.5 --timeout 3.0 --ramp-down 6

# optional: log esc_status only
./scripts/px4_bdshot_capture.py --port /dev/ttyACM2 --duration 40 -o runs/px4-cap.csv
```

Details: [setup_px4_bdshot.md](setup_px4_bdshot.md) (formerly `bdshot_baseline.md`).

## Switching between setups

1. **Power down** ESC / motor bus.
2. **Move the signal wire** (stand ESC out ↔ FPV motor pad). Never parallel both.
3. Use the matching **rig config** and **command path** from the table above.
4. For SETUP B on a battery, prefer soft ramp-down; on a 3 A brick, hard stop
   can fault the supply (see PX4 docs).

## What is shared

- Same AM32 target / `HWCI_PERF=1` firmware (BDShot counters are zero-cost when
  idle if the host never enables BDShot).
- Same ST-Link debug header.
- Same motor/ESC hardware (wiring of the **signal** pin differs).
