/*
 * bemf_zc.c - extracted from main.c (behavior-neutral split)
 */

#include "bemf_zc.h"
#include "motor_runtime.h"
#include "commutation.h"
#include "main.h"
#include "common.h"
#include "comparator.h"
#include "phaseouts.h"
#include "targets.h"
#include "IO.h"
#include "peripherals.h"
#include "functions.h"
#include "eeprom.h"
#include "hwci_perf.h"

/*
 * Missed-ZC fallback (BLHeli-style timeout commutation). After each
 * commutation COM_TIMER is re-armed as a deadline for the next crossing:
 * expected arrival plus 50% grace. An accepted crossing re-arms COM_TIMER
 * for the commutation schedule (cancelling the deadline); if the deadline
 * fires instead, commutate blind at extrapolated timing and keep watching.
 * A missed crossing costs one extrapolated step, not a mode change - there
 * is no fallback to poll mode at runtime.
 */
#define ZC_BLIND_STEP_LIMIT 8 /* consecutive extrapolated steps before the stall rail takes over */

RAM_FUNC void PeriodElapsedCallback()
{
	uint8_t blind = 0;
	DISABLE_COM_TIMER_INT(); // disable interrupt
	if (zc_deadline_armed) {
		// Deadline firing, not a commutation scheduled by an accepted
		// zero-cross (interruptRoutine cancels the deadline first).
		zc_deadline_armed = 0;
		if (!running || old_routine) {
			return; // stale deadline after a stop or forced restart
		}
		if (zc_blind_steps >= ZC_BLIND_STEP_LIMIT) {
			// Position unknown for too long: stop stepping blind and let
			// the INTERVAL_TIMER stall rail / desync machinery restart
			// through the normal startup path.
			maskPhaseInterrupts();
			return;
		}
		blind = 1;
		zc_blind_steps++;
		zc_blind_steps_total++;
		// Take the full elapsed time as the (late) crossing measurement
		// and commutate now. The inflated interval feeds the average so
		// timing hunts slower - the safe direction for a decelerating
		// rotor - and the next accepted crossing resyncs immediately.
		maskPhaseInterrupts();
		uint32_t elapsed = INTERVAL_TIMER_COUNT;
		if (elapsed > 65535u) {
			elapsed = 65535u;
		}
		lastzctime = thiszctime;
		thiszctime = (uint16_t)elapsed;
		SET_INTERVAL_TIMER_COUNT(0);
	}
	commutate();
	commutation_interval = ((commutation_interval) + ((lastzctime + thiszctime) >> 1)) >> 1;
	if (!eepromBuffer.auto_advance) {
		advance = (commutation_interval * temp_advance) >> 6; // 60 divde 64 0.9375 degree increments
	} else {
		advance = (commutation_interval * auto_advance_level) >> 6; // 60 divde 64 0.9375 degree increments
	}
	waitTime = (commutation_interval >> 1) - advance;
	if (!old_routine) {
		enableCompInterrupts(); // enable comp interrupt
		// Next crossing expected (commutation_interval - waitTime) from
		// now; arm the blind-step deadline at expected + 50% grace.
		uint32_t deadline = (commutation_interval - waitTime) + (commutation_interval >> 1);
		if (deadline > 65535u) {
			deadline = 65535u;
		}
		zc_deadline_armed = 1;
		SET_AND_ENABLE_COM_INT((uint16_t)deadline);
	}
	if (!blind && zero_crosses < 10000) {
		zero_crosses++;
	}
	HWCI_PERF_ZC();
}

RAM_FUNC void interruptRoutine()
{
	HWCI_PERF_ZC_PHASE_CAPTURE(); // TIM1 phase at ISR entry, pre-confirm
#ifdef MCU_F051
	// PWM phase (TIM1 CNT) at the edge, sampled before the confirm loop's
	// wall-clock window advances it: the turn-on-pileup compensation below
	// keys off where the comparator edge actually appeared, not where it was
	// finally accepted.
	uint16_t zc_pwm_cnt = (uint16_t)TIM1->CNT;
#endif
	//   if (average_interval > 125) {
	//        if ((INTERVAL_TIMER_COUNT < 125) && (duty_cycle < 600) && (zero_crosses < 500)) { // should be impossible, desync?exit anyway
	//           return;
	//        }
	//        stuckcounter++; // stuck at 100 interrupts before the main loop happens
	//                        // again.
	//        if (stuckcounter > 100) {
	//            maskPhaseInterrupts();
	//            zero_crosses = 0;
	//            return;
	//        }
	//    }
	// Zero-cross confirm: reject unless the window's reads hold the
	// post-crossing level. Loop speed sets the sampling window: inlining
	// getCompOutputLevel removes the per-sample call overhead, and with the
	// companion RAM-execution change the loop is faster still; filter_level
	// is scaled (42/10/7 on F051) to keep the same wall-clock window as the
	// stock ~56-cycle sampling cadence.
#ifdef MCU_F051
	// Glitch-tolerant variant: the ~16-cycle cadence lands on a brief
	// comparator glitch ~3x more often than stock sampling, and a strict
	// all-samples-must-agree confirm defers detection to the NEXT
	// comparator edge - up to a PWM period late. Bench-measured on the
	// ARK 4IN1: 2-3x higher commutation jitter at 15-20 kHz commutation
	// rates vs stock cadence (upstream 2.1% vs 5.4% at full throttle).
	// Tolerating up to filter_level/4 bad samples per window accepts
	// through isolated glitches while a genuinely un-crossed level still
	// rejects via the early-out; the full window length (and so the
	// sustained-noise immunity of the filter_level retune) is unchanged.
	{
		int bad = 0;
		const int tolerance = filter_level >> 2;
		for (int i = 0; i < filter_level; i++) {
			if (getCompOutputLevel() == rising) {
				if (++bad > tolerance) {
					HWCI_PERF_CONFIRM_REJECT();
					return;
				}
			}
		}
	}
#else
	for (int i = 0; i < filter_level; i++) {
#	if defined(MCU_F031) || defined(MCU_G031)
		if (((current_GPIO_PORT->IDR & current_GPIO_PIN) == !(rising))) {
#	else
		if (getCompOutputLevel() == rising) {
#	endif
			HWCI_PERF_CONFIRM_REJECT();
			return;
		}
	}
#endif
#ifdef MCU_F051
	// Turn-on-pileup timestamp compensation. A zero-cross that physically
	// occurs during the PWM off-window (freewheel) is invisible to the
	// comparator and is only registered at the next turn-on, quantizing the
	// timestamp to the PWM grid; those corrupted intervals feed waitTime and
	// the commutation loop hunts, amplifying the quantization into the
	// measured jitter hump (bench: ~23% at t60/t70 where the commutation rate
	// beats against the 24 kHz PWM). Detections of such crossings land 1-3
	// phase bins after turn-on (bin 1 peak = comparator+ISR latency of
	// ~1.3-2.6 us; bin 0 stays at the uniform rate) and the true crossing lay
	// on average half the off-window earlier - back-date thiszctime by that
	// half-window so the loop tracks the estimated true crossing instead of
	// the grid position.
	//
	// Guards, each tied to a bench-observed failure of the unguarded version
	// (which cut hump jitter 23.4->17.9% but desynced on a throttle step and
	// doubled low-RPM jitter):
	//  - correct thiszctime ONLY; the interval timer still resets to zero at
	//    the detection instant, so one bad estimate perturbs one interval and
	//    cannot cascade into the next (the cascading variant lost sync at 70A)
	//  - duty-slew guard: skip while adjusted_duty_cycle is moving, i.e.
	//    during throttle transients, where the loop must track real
	//    acceleration rather than have timestamps rewritten under it
	//  - hump band only (commutation interval < 2 PWM periods): at low RPM the
	//    interval dwarfs the PWM period, there is no hunting to break, and the
	//    correction only adds variance
	//  - clamp to interval/8 as a backstop against any degenerate state
	// Units: INTERVAL_TIMER (TIM2) is 2 MHz and TIM1 is 48 MHz, so one INTERVAL
	// tick is 24 TIM1 ticks; half an off-window of N TIM1 ticks is N/48 INTERVAL
	// ticks, computed as (N * 1365) >> 16 to keep a soft divide out of the ISR.
	uint16_t zc_grid_comp = 0;
	{
		static uint16_t zc_prev_duty;
		const uint16_t zc_arr = tim1_arr;
		const uint16_t zc_duty = adjusted_duty_cycle;
		const uint16_t zc_slew =
			(zc_duty >= zc_prev_duty) ? (uint16_t)(zc_duty - zc_prev_duty) : (uint16_t)(zc_prev_duty - zc_duty);
		zc_prev_duty = zc_duty;
		const uint32_t zc_ci = commutation_interval;
		if (zero_crosses >= 100 && zc_duty < zc_arr					  /* off-window exists */
		    && zc_pwm_cnt >= (uint16_t)(zc_arr >> 5)					  /* pile-up bins 1-3 */
		    && zc_pwm_cnt < (uint16_t)(zc_arr >> 3) && zc_slew <= (uint16_t)(zc_arr >> 8) /* duty steady */
		    && (zc_ci * 12u) < (uint32_t)zc_arr + 1u					  /* hump band only */
		) {
			uint32_t comp = ((uint32_t)(zc_arr - zc_duty) * 1365u) >> 16;
			const uint32_t cap = zc_ci >> 3;
			if (comp > cap) {
				comp = cap;
			}
			zc_grid_comp = (uint16_t)comp;
		}
	}
#endif
	__disable_irq();
	maskPhaseInterrupts();
	zc_deadline_armed = 0; // COM_TIMER now times commutation, not the missed-ZC deadline
	zc_blind_steps = 0;
	lastzctime = thiszctime;
#ifdef MCU_F051
	thiszctime = (uint16_t)(INTERVAL_TIMER_COUNT - zc_grid_comp);
#else
	thiszctime = INTERVAL_TIMER_COUNT;
#endif
	SET_INTERVAL_TIMER_COUNT(0);
	SET_AND_ENABLE_COM_INT(waitTime + 1); // enable COM_TIMER interrupt
	__enable_irq();
	HWCI_PERF_ZC_PHASE_COMMIT(); // accepted edge: bin its PWM phase
}

void startMotor()
{
	if (running == 0) {
		commutate();
		commutation_interval = 10000;
		SET_INTERVAL_TIMER_COUNT(5000);
		running = 1;
	}
	DISABLE_COM_TIMER_INT(); // a stale blind-step deadline must not commutate the restart
	zc_deadline_armed = 0;
	zc_blind_steps = 0;
	enableCompInterrupts();
}
