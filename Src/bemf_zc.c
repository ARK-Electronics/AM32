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
/*
 * Miss-RATE rail. zc_blind_steps only counts CONSECUTIVE misses - an
 * alternating real/missed pattern (the demag signature) resets it on every
 * accepted crossing and would blind-step indefinitely at ~12.5% interval
 * inflation per miss, never reaching the step limit, the trust rail (real
 * crossings keep the average under the changeover band) or the stall rail
 * (both paths reset INTERVAL_TIMER). Leaky bucket: each blind step adds
 * ZC_MISS_BUCKET_INC, each accepted crossing drains 1, at
 * ZC_MISS_BUCKET_LIMIT the handler stops stepping blind and hands the loop
 * to the stall rail exactly like the consecutive-step limit. Sustained miss
 * rates above 1-in-4 climb; 8 consecutive misses reach the limit on the
 * same deadline event as ZC_BLIND_STEP_LIMIT, so the consecutive case is
 * unchanged. Alternating 50% trips after ~12 misses (~4 electrical revs).
 */
#define ZC_MISS_BUCKET_INC 3
#define ZC_MISS_BUCKET_LIMIT 24
/*
 * Blind stepping requires an ESTABLISHED closed loop. At the poll->interrupt
 * handoff commutation_interval is whatever the startup noise made of it -
 * bench: noise-shrunk to ~1300 ticks while the true interval of the barely
 * moving rotor is 5-10x longer - so a deadline at 1.5x the believed interval
 * fires BEFORE the first real crossing can arrive, commutates blind, and
 * drags the stator field away from the rotor. The result is a stable false
 * lock: noise bursts get accepted as crossings, blind steps bridge the gaps
 * between bursts (bench: ~40% of commutations blind, phantom ~15k eRPM,
 * rotor stalled at 3 A), and the stall rail almost never fires because every
 * blind step resets INTERVAL_TIMER. Below this zero-cross count the handler
 * keeps legacy semantics (commutate once per accepted crossing, no re-arm):
 * a noise chain then dies at the 45000-tick stall rail and the poll path
 * kick-steps the rotor exactly as ark-release does. 100 matches the startup
 * self-heal window in commutate() and the "stable running" gates elsewhere;
 * zero_crosses resets on desync/stop, so every re-acquisition passes through
 * the legacy path again before blind stepping re-arms.
 */
#define ZC_DEADLINE_MIN_ZC 100

RAM_FUNC void PeriodElapsedCallback()
{
	uint8_t blind = 0;
	DISABLE_COM_TIMER_INT(); // disable interrupt
	if (!running || old_routine) {
		// COM events are meaningless outside interrupt closed-loop: a
		// stale blind-step deadline after a stop or forced restart, or a
		// poll-mode COM_TIMER ARR write landing while the deadline's
		// interrupt enable was still set. Legacy never ran this handler
		// in poll mode (the enable stayed off); keep that invariant, or
		// the cancel-race guard below re-arms COM against the poll
		// startup and spurious commutations fight zcfoundroutine.
		zc_deadline_armed = 0;
		return;
	}
	if (zc_deadline_armed) {
		// Deadline firing, not a commutation scheduled by an accepted
		// zero-cross (interruptRoutine cancels the deadline first).
		zc_deadline_armed = 0;
		if (zc_blind_steps >= ZC_BLIND_STEP_LIMIT || zc_miss_bucket >= ZC_MISS_BUCKET_LIMIT) {
			// Position unknown for too long (consecutively or as a
			// sustained miss rate): stop stepping blind. Kick the
			// INTERVAL_TIMER past the 45000 stall threshold so the
			// main-loop rail (faultHandleBemfIntervalStall) restarts
			// through the startup path on its next pass instead of
			// 22 ms from now. That rail also charges the cross-episode
			// desync bucket - do NOT charge here as well (it double-
			// counted the episode), and the stall rail is guaranteed to
			// see it: comparator interrupts are masked, so no accepted
			// crossing can reset INTERVAL_TIMER before the main loop.
			maskPhaseInterrupts();
			SET_INTERVAL_TIMER_COUNT(46000);
			return;
		}
		blind = 1;
		zc_blind_steps++;
		zc_miss_bucket += ZC_MISS_BUCKET_INC;
		if (zc_blind_window_count < 255) {
			zc_blind_window_count++; // grind-rate window (faults.c, 1 kHz)
		}
		HWCI_PERF_BLIND_STEP();
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
	} else {
		// Cancel race: SET_AND_ENABLE_COM_INT clears the peripheral flag,
		// but a deadline event that reached NVIC pending just before an
		// accepted zero-cross cancelled it still runs this handler once.
		// The genuine commutation fires with INTERVAL_TIMER at ~waitTime
		// (both reset together at the crossing); far earlier means this
		// entry is that stale pend - re-arm the remainder and bail.
		uint32_t t = INTERVAL_TIMER_COUNT;
		if (t + 4u < waitTime) {
			SET_AND_ENABLE_COM_INT((uint16_t)(waitTime - t));
			return;
		}
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
		if (zero_crosses >= ZC_DEADLINE_MIN_ZC) {
			// Next crossing expected (commutation_interval - waitTime)
			// from now; arm the blind-step deadline at expected + 50%
			// grace. The 16-bit clamp shrinks the grace above CI ~40000,
			// but the trust rail restarts long before a healthy loop
			// runs that slow.
			uint32_t deadline = (commutation_interval - waitTime) + (commutation_interval >> 1);
			if (deadline > 65535u) {
				deadline = 65535u;
			}
			zc_deadline_armed = 1;
			SET_AND_ENABLE_COM_INT((uint16_t)deadline);
		}
	}
	if (!blind && zero_crosses < 10000) {
		zero_crosses++;
	}
	if (!blind) {
		// Blind steps must not feed the jitter accumulators: the
		// extrapolated 1.5x interval is not a measured crossing, and the
		// perf gate (zero_crosses >= 100) is the same window in which
		// blind stepping is armed - counting them would conflate real
		// jitter with blind-step count in exactly the runs being read.
		// Blind steps have their own counter (HWCI_PERF_BLIND_STEP).
		HWCI_PERF_ZC();
	}
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
	// Demag-late crossing detection, decision half (sampler:
	// runtimeSampleBemfPreLevel). No pre-crossing dwell observed since the
	// last commutation means this edge is the freewheel demag clamp
	// releasing over an already-past crossing - the loop is commutating
	// late by the demag duration. Late drive raises current and more
	// current lengthens demag, so left alone this settles into a stable
	// mistimed lock (bench 2026-07-22: ~30% slow at 8-10x current on ~1/3
	// of warm snap starts, 100% of hot ones; never self-heals; invisible
	// to the desync jump check and the trust rail because the interval is
	// steady and fast-looking). Charge the miss bucket and let the
	// consecutive counter drive the same power cut as blind steps
	// (control_loop): less current shortens demag, the pre-level dwell
	// reappears and the loop re-times itself - the firmware equivalent of
	// the throttle-blip escape verified on the bench. Sustained demag-late
	// accepts escalate to the stall rail exactly like a sustained miss
	// rate. Gated to CI > 500 (250 us+), where the healthy pre-level dwell
	// spans many main-loop passes so the sampler cannot miss it.
	//
	// The response is the power cut ONLY - no miss-bucket charge, no stall
	// rail. Normal spool-up under load is genuinely demag-late for long
	// stretches (high slip current), so any restart escalation turns every
	// hard start into a kick loop (bench: 2-3 restarts and ~10 desyncs per
	// 2 s start attempt with bucket escalation enabled). The cut is
	// self-resolving in both cases: during spool-up it bounds accel
	// current until slip drops, and in the mistimed lock it collapses the
	// current that sustains the wrong timing. A loop that stays demag-late
	// forever just stays current-limited - strictly better than the 8-10x
	// current of the untreated lock.
	if (commutation_interval > 500 && !zc_pre_seen) {
		zc_demag_accepts++;
		if (zc_demag_run < 255) {
			zc_demag_run++;
		}
	} else {
		zc_demag_run = 0;
	}
	zc_pre_seen = 0;
	if (zc_miss_bucket) {
		zc_miss_bucket--; // accepted crossing drains the miss-rate bucket
	}
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
		DISABLE_COM_TIMER_INT(); // a stale blind-step deadline must not commutate the restart
		zc_deadline_armed = 0;
		zc_blind_steps = 0;
		zc_miss_bucket = 0;
		zc_blind_window_count = 0;
		zc_pre_seen = 1;
		zc_demag_run = 0;
		commutate();
		commutation_interval = 10000;
		SET_INTERVAL_TIMER_COUNT(5000);
		running = 1;
	}
	enableCompInterrupts();
}
