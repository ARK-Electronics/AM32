/*
 * runtime_loop.h - main-loop motor policies (behavior-neutral extract)
 *
 * Variable PWM, desync, IRQ priority, telemetry send, ADC/LVC, running
 * limits, and sine/stepper branch previously inlined in main()'s while(1).
 *
 * Keep these as out-of-line functions: inlining desync/variable-PWM into
 * main() was observed to provoke high-throttle free-run stalls on F051.
 * Small helpers that are safe to inline live in faults.h / functions.h.
 */
#ifndef RUNTIME_LOOP_H_
#define RUNTIME_LOOP_H_

#include <stdint.h>

/* Recompute TIM1 ARR from commutation rate; updates *last_tim1_arr. */
void runtimeUpdateVariablePwm(uint16_t *last_tim1_arr);

/* Desync detect from average interval jump. */
void runtimeProcessDesyncCheck(void);

/* Sample the comparator for the pre-crossing dwell (demag-late detection). */
void runtimeSampleBemfPreLevel(void);

/* DShot vs COM/COMP IRQ priority when telemetry is active. */
void runtimeUpdateDshotIrqPriority(void);

/* Serial telemetry / ESC info packets. */
void runtimeSendTelemetryIfNeeded(void);

/* ADC sample + battery LVC + optional ADC input path. */
void runtimeProcessAdcAndProtections(void);

/*
 * Brushless running path (limits, filter, stall) or sine/stepper branch,
 * matching the previous if (stepper_sine == 0) / else structure.
 */
void runtimeMotorModeTick(void);

#endif /* RUNTIME_LOOP_H_ */
