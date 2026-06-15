/*
 * profiling.h
 *
 * Lightweight ISR cycle profiling for the flash-/cycle-constrained F051.
 *
 * Cortex-M0 has no DWT->CYCCNT and no ITM/SWO, so the two canonical ways to
 * measure ISR cost are (a) a GPIO toggled around the ISR, read on a logic
 * analyzer / scope, and (b) a free-running timer used as a poor-man's cycle
 * counter. This header does BOTH from one instrumentation point:
 *
 *   - GPIO marker: a pin is driven HIGH while any instrumented ISR runs, so the
 *     wire shows aggregate ISR (CPU) load. Optional; needs a free pad.
 *   - Cycle counter: TIM17 (otherwise unused on the F051) is repurposed as a
 *     48 MHz free-running counter -> 1 tick == 1 core cycle. Per-ISR last/max
 *     cycle counts land in arrays you read over SWD (watch prof_cyc_max[]) or
 *     stream over telemetry.
 *
 * All of this compiles to nothing unless PROFILE_ISR is defined, so production
 * builds are byte-for-byte identical.
 *
 * Caveat: a measured duration includes any higher-priority ISR that preempts
 * the region. The comparator zero-cross is the highest-priority motor ISR, so
 * its numbers are clean; lower-priority regions read as wall-clock (which is
 * usually what you want for a "can it keep up" budget).
 */

#ifndef PROFILING_H_
#define PROFILING_H_

/* ---- turn the harness on for a measurement build -------------------------
 * Define here, per-target in targets.h, or pass -DPROFILE_ISR to make.       */
// #define PROFILE_ISR

/* ---- optional logic-analyzer marker pin ----------------------------------
 * Point these at a pad broken out on your board (e.g. the serial-telemetry
 * pin with telemetry disabled for the run). Leave undefined for counters-only
 * measurement over SWD.                                                       */
// #define PROFILE_GPIO_PORT GPIOB
// #define PROFILE_GPIO_PIN  LL_GPIO_PIN_6

#ifdef PROFILE_ISR
#include "main.h"

enum {
    PROF_COMP_ZC = 0, /* ADC1_COMP_IRQHandler -> interruptRoutine (zero-cross) */
    PROF_20KHZ,       /* TIM6_DAC_IRQHandler  -> tenKhzRoutine    (control)    */
    PROF_DSHOT,       /* EXTI4_15_IRQHandler  -> processDshot     (decode)     */
    PROF_TIM14,       /* TIM14_IRQHandler     -> PeriodElapsedCallback         */
    PROF_N
};

extern volatile uint16_t prof_cyc_last[PROF_N]; /* last duration, core cycles  */
extern volatile uint16_t prof_cyc_max[PROF_N];  /* worst case since boot       */
extern volatile uint32_t prof_cyc_sum[PROF_N];  /* running cycle total (wraps) */
extern volatile uint32_t prof_calls[PROF_N];    /* invocation count            */

void profiling_init(void); /* repurposes TIM17 as a 48 MHz counter + marker pin */

__attribute__((always_inline)) static inline uint16_t prof_enter(void)
{
#ifdef PROFILE_GPIO_PORT
    PROFILE_GPIO_PORT->BSRR = PROFILE_GPIO_PIN; /* marker high */
#endif
    return (uint16_t)TIM17->CNT;
}

__attribute__((always_inline)) static inline void prof_exit(int i, uint16_t t0)
{
    uint16_t d = (uint16_t)((uint16_t)TIM17->CNT - t0);
#ifdef PROFILE_GPIO_PORT
    PROFILE_GPIO_PORT->BRR = PROFILE_GPIO_PIN; /* marker low */
#endif
    prof_cyc_last[i] = d;
    if (d > prof_cyc_max[i])
        prof_cyc_max[i] = d;
    prof_cyc_sum[i] += d;
    prof_calls[i]++;
}

/* Wrap a region: PROF_ENTER(PROF_20KHZ); ...work...; PROF_EXIT(PROF_20KHZ); */
#define PROF_ENTER(i) uint16_t _pt_##i = prof_enter()
#define PROF_EXIT(i)  prof_exit((i), _pt_##i)

#else /* !PROFILE_ISR */
#define profiling_init() ((void)0)
#define PROF_ENTER(i) ((void)0)
#define PROF_EXIT(i)  ((void)0)
#endif

#endif /* PROFILING_H_ */
