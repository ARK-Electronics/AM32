# Demag compensation — bench validation findings (PR #24)

Hardware: ARK 4IN1 (STM32F051), JS Technology 2306 1800KV (12N14P), Flight
Stand 50, ST-Link V3 over SWD. Firmware built with `HWCI_PERF=1`.

**Verdict: land the comp-OFF path; do NOT enable demag compensation. The
compensation, as implemented, is net-harmful under load and needs a rework
before it is enabled on hardware.**

Demag compensation is off by default (eeprom bytes 184/185 default `0xff` →
0), so this branch is safe to merge as-is: the feature ships compiled-in but
dormant. The `DEMAG_COMP=<1|2|3>` build knob is the only way to force it on,
and should stay a bench-only experiment until the rework below lands.

## Result 1 — inertness A/B (no prop, 25.0 V / 3.0 A supply): PASS

Both sides ran `noprop_smoke_3a` (10 %/20 % throttle, 2.8 A abort). **A** =
ark-release HEAD (7d3b771, no demag code, perf v4). **B** = this branch (perf
v5, demag compiled in but off). The question: does the compiled-in-but-inert
demag port perturb normal operation?

| metric              | A: ark-release | B: demag (off) |
|---------------------|---------------:|---------------:|
| zc jitter, mean     | 2.31 %         | 2.35 %         |
| zc jitter, max      | 12.2 %         | 12.2 %         |
| peak current        | 0.51 A         | 0.51 A         |
| CPU load, max       | 62.2 %         | 61.1 %         |
| idle loop rate      | 55.1 kHz       | 52.1 kHz       |
| demag / bemf-timeout| 0 / 0          | 0 / 0          |

Zero-cross jitter is flat (the load-bearing "port adds no jitter" metric). The
idle-loop-rate dip is the compiled-in `HAS_DEMAG_COMP` checks; CPU load did not
rise, so there is no systematic regression and ~40 % headroom remains. n=1 per
side, so treat the small deltas as run-to-run noise and the flat jitter as the
signal. Runs: `runs/inertness-ab-arkrelease`, `runs/demag-rebase-inertness-smoke`.

## Result 2 — demag-event A/B (HQ5136 prop + 6S battery): comp-ON net-harmful

Both sides ran `demag_step_stress` (snap-steps to 90/95/100 %, 55 A abort).
Only the feature flag differed. Peak current ~38 A both sides (partly
discharged pack; profile expects 42–49 A fresh).

| step100 (100 % throttle) | comp OFF | comp ON (DEMAG_COMP=2) |
|--------------------------|---------:|-----------------------:|
| mean stand RPM           | ~28,650  | **~8,500 (collapsed)**  |
| samples < 60 % of max    | 17 / 400 | **321 / 400**           |
| bemf-timeout samples (run)| **0**   | **595**                 |
| host desync events        | 0       | 3                       |
| fw_demag_events           | 0       | 290                     |

With compensation **off** the motor rode the snap-steps cleanly; with it **on**
it lost sync hard — at 100 % throttle it collapsed from ~30,000 RPM to a mean
of ~8,500 while still commanding full throttle. The feature induces the very
desync it is meant to prevent. Runs: `runs/demag-stress-off`,
`runs/demag-stress-on`.

## Root cause

Verified in code and run data (`runs/demag-stress-on`):

- **Ungated, unconfirmed demag edge path.** The F051 comparator ISR
  ([Mcu/f051/Src/stm32f0xx_it.c](../Mcu/f051/Src/stm32f0xx_it.c), the
  `auto_blanking` branch) accepts the first comparator edge after unmask and
  calls `demagEdgeRoutine()` with **no** `INTERVAL_TIMER > average_interval/2`
  gate and **no** confirm loop — both of which protect the normal zero-cross
  path right below it. During the high-current snap transient the demag edge
  fires on noise/commutation ringing.
- **Arm gate has no stability check.** `interruptRoutine()` arms
  `auto_blanking` whenever `demag_comp_level && input>400 &&
  commutation_interval>100`, so it arms every commutation straight through a
  fast acceleration — exactly when timing margin is thinnest.
- **Aggressive response.** Each detected demag event does `allOff()` + duty cut
  + a **re-entrant `interruptRoutine()`** from ISR context, which kicks the
  bridge off and forces a commutation mid-recovery, sustaining the desync.

What the data shows: both OFF and ON accelerate identically and cleanly
(commutation_interval decreasing smoothly) until the interval reaches ~120
ticks (high RPM, near the `commutation_interval>100` arm boundary). OFF
continues to ~100 and holds; ON's cadence collapses within one 5 ms sample
(interval 120→541→11,690, bemf timeout), coincident with the first
demag_events. At 200 Hz perf sampling the demag-edge mistiming and the
`allOff`/cut/re-enter response fall in the same window, so their ordering is
not resolvable from this data — both are implicated. (Note: the
`e_rpm/(pole_pairs·stand_rpm)` "sync lead" seen early in the step is a
stand-RPM-estimator lag artifact present in BOTH runs, not a demag effect; use
`commutation_interval` + bemf flag as the discriminator.)

## Fix path (for the rework, tracked separately from this PR)

1. **Stability gate on the arm** (highest leverage, lowest risk): only arm when
   `input` high **and** `commutation_interval > floor` **and** `zero_crosses`
   stable **and** recent commutation-interval slew below a threshold. Demag
   compensation only matters at sustained load; disarm during transients.
2. **Gate/confirm the demag edge** like the normal ZC path (minimum-time gate +
   a confirm sample) so noise cannot trigger it.
3. **Soften the response**: rate-limit the cut and drop the re-entrant
   `interruptRoutine()` call.
4. **Altitude**: route demag-release detection through the same glitch-tolerant
   confirm discipline as the normal ZC instead of a parallel ungated edge path.

Suggested next bench experiment: an **observe-only** build (measure
`blanking_len`/`demag_events` but skip `allOff`/cut/re-enter) plus the
stability gate, to gather stage-ordering evidence safely.

## Fixes already applied in this PR (from the code review)

These are correct and were validated by the inertness A/B; they are not the
cause of Result 2 (a feature-level flaw, above):

- `blanking_length` measured against a snapshotted scheduling delay
  (`demag_wait_ticks`) instead of the next-cycle `waitTime`.
- Adaptive bemf stall timeout clamped to 45000 (the 16-bit INTERVAL_TIMER would
  otherwise make the threshold unreachable at low RPM).
- Demag duty cut handed to `tenKhzRoutine` via a latch/pending flag instead of
  a direct write raced by the slew write-back.
- Stale comparator EXTI pending cleared before unmask (all ported MCUs).
- `resetDemagState()` clears demag state before `commutate()` on start, and at
  the desync / bemf-timeout sites.
- Makefile flag-stamp so `HWCI_PERF`/`DEMAG_COMP` changes force a rebuild.
