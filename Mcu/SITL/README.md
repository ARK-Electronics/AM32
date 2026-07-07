# AM32 SITL (software in the loop)

Runs the AM32 firmware as a native Linux executable against a simulation
of the motor, bridge and battery, with DroneCAN input/output over
multicast UDP. This allows testing of the firmware logic (startup,
commutation, DroneCAN protocol, parameters) without ESC hardware.

## Building

```
make AM32_SITL_CAN
```

produces `obj/AM32_AM32_SITL_CAN_<version>.elf`, a normal Linux
executable.

## Running

```
obj/AM32_AM32_SITL_CAN_*.elf --node-id 10 --verbose
```

Options:

- `--config FILE` JSON file with motor/battery/esc/sim properties (see
  `example.json`, all keys optional)
- `--eeprom FILE` eeprom backing file (default `am32_eeprom.bin`). A
  missing file is seeded with the AM32 configurator default settings
- `--can-uri URI` CAN interface, default `mcast:0` (group 239.65.82.N
  port 57732, wire compatible with libcanard and ArduPilot SITL)
- `--node-id N` force the DroneCAN node ID, otherwise DNA is used
- `--speedup X` simulation speed relative to wall clock, 0 = free running
- `--uid STR` string used to derive the 16 byte unique ID
- `--verbose` 1Hz state line on stderr
- `--nosleep` busy wait instead of sleeping. Uses two full CPU cores but
  avoids OS sleep/wakeup latency for the most accurate wall clock pacing

The virtual ESC can then be controlled with the DroneCAN GUI tool or
pydronecan on `mcast:0`. A test script is included:

```
python3 Mcu/SITL/sitl_can_test.py --throttle 0.5 --duration 20
```

which arms, ramps the throttle via `esc.RawCommand` and reports the
`esc.Status` telemetry (RPM, voltage, current, temperature).

Note that with no DroneCAN traffic directed at the node it will reboot
every 2 seconds from the firmware signal-timeout logic, exactly as real
hardware does. Reboots (including `RestartNode` and watchdog resets)
re-exec the process; the eeprom file persists.

Each instance holds a lock on its eeprom file: two instances sharing an
eeprom (and therefore a node ID) would interleave their DroneCAN
transfers on the bus, which shows up as erratic telemetry. To run
multiple ESCs give each its own `--eeprom` and `--node-id`.

## Architecture

The firmware runs unmodified (built as `am32_main()`) in one thread. A
simulation thread owns simulated time, advancing it in fixed physics
steps (500ns default) and delivering emulated interrupts (BEMF
comparator, commutation timer, 20kHz loop timer, CAN RX) by suspending
the firmware thread with a signal and running the handler, reproducing
the run-to-completion interrupt semantics of the real MCU. Simulated
time is decoupled from wall time and paced to `--speedup`.

The emulated MCU follows the G431 target: TIM1 PWM generation with
preloaded ARR/CCR, a 2MHz interval timer, one-shot commutation timer,
20kHz loop timer and 1MHz utility timer, plus comparator/EXTI blanking
behaviour matching `Mcu/g431`. Timer reads from interrupt context step
the physics, so `delayMicros()` inside handlers and the comparator
filter loop behave as on hardware.

The motor model (`sim/motor.c`) is a trapezoidal back-EMF BLDC model
based on open-bldc-csim: per-phase currents, solved star point voltage
(so the floating phase terminal voltage and its zero crossings are
physical), fet Rds_on, body diode clamping of a floating phase carrying
current, and battery voltage sag from internal resistance. The
comparator compares the floating phase against the virtual neutral with
configurable noise and hysteresis, so the firmware's blanking and
filtering logic is genuinely exercised at PWM switching level.
