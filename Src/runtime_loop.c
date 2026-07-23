/*
 * runtime_loop.c - extracted from main.c (behavior-neutral)
 */

#include "runtime_loop.h"

#include "main.h"
#include "common.h"
#include "motor_runtime.h"
#include "faults.h"
#include "esc_state.h"
#include "control_loop.h"
#include "commutation.h"
#include "functions.h"
#include "peripherals.h"
#include "phaseouts.h"
#include "comparator.h"
#include "eeprom.h"
#include "signal.h"
#include "dshot.h"
#include "targets.h"
#include "ADC.h"
#include "adc_app.h"
#include "kiss_telemetry.h"
#include "IO.h"

#ifdef USE_SERIAL_TELEMETRY
#	include "serial_telemetry.h"
#endif
/* SITL always provides send_telem_DMA; non-SITL needs USE_SERIAL_TELEMETRY. */
#ifdef MCU_SITL
#	include "serial_telemetry.h"
#endif

/* Zero-cross filter levels (were file-local defines in main.c) */
#ifdef MCU_F051
#	define ZC_FILTER_MAX 42
#	define ZC_FILTER_RUN_MIN 10
#	define ZC_FILTER_FAST 7
#else
#	define ZC_FILTER_MAX 12
#	define ZC_FILTER_RUN_MIN 3
#	define ZC_FILTER_FAST 2
#endif

void runtimeUpdateVariablePwm(uint16_t *last_tim1_arr)
{
	uint16_t next_tim1_arr = tim1_arr;    // unchanged unless variable_pwm recomputes it below
	if (eepromBuffer.variable_pwm == 1) { // uses range defined by pwm frequency setting
		next_tim1_arr = map(commutation_interval, 96, 200, TIMER1_MAX_ARR / 2, TIMER1_MAX_ARR);
	}
	if (eepromBuffer.variable_pwm == 2) { // uses automatic range
		if (average_interval < 250 && average_interval > 100) {
			next_tim1_arr = average_interval * (CPU_FREQUENCY_MHZ / 9);
		}
		if (average_interval < 100 && average_interval > 0) {
			next_tim1_arr = 100 * (CPU_FREQUENCY_MHZ / 9);
		}
		if ((average_interval >= 250) || (average_interval == 0)) {
			next_tim1_arr = 250 * (CPU_FREQUENCY_MHZ / 9);
		}
	}
	if (next_tim1_arr != *last_tim1_arr) {
		*last_tim1_arr = next_tim1_arr;
		// recompute the scale at idle priority, outside the mask so no interrupt is
		// held off across the divide; then publish tim1_arr and the scale together so
		// the 20khz routine never pairs a new arr with a stale scale
		const uint32_t next_scale = ((uint32_t)next_tim1_arr << 16) / 2000;
		__disable_irq();
		tim1_arr = next_tim1_arr;
		pwm_to_arr_scale_q16 = next_scale;
		__enable_irq();
	}
}

void runtimeProcessDesyncCheck(void)
{
	static uint8_t slow_avg_revs;
	average_interval = e_com_time / 3;
	// Any external zero_crosses reset (stall rail, stop path, bidir
	// reversal) closes the evaluation gate below with slow_avg_revs
	// holding a partial count; the leftover would then complete the
	// trust-rail rev gate on the first evaluation of the NEXT closed-loop
	// run, against a still-lagging average_interval. zero_crosses climbs
	// one crossing at a time, so every re-acquisition passes through this
	// window and clears the counter.
	if (zero_crosses <= 10) {
		slow_avg_revs = 0;
	}
	if (desync_check && zero_crosses > 10) {
		uint8_t desynced = (getAbsDif(last_average_interval, average_interval) > average_interval >> 1) &&
				   (average_interval < 2000); // throttle resitricted before zc 20.
		// Interrupt-ZC trust rail: with no per-commutation fallback to poll
		// mode, a closed loop tracking artifact edges below usable BEMF
		// (stable false lock: crossings keep arriving, so neither the
		// blind-step deadline nor the jump check above can see it) needs a
		// way out. Sustained slow averages hand back to the poll path
		// legacy-style - soft, no desync accounting - so a throttle chop
		// decelerating through the band behaves exactly as before. A
		// genuine false lock has a stationary rotor: poll then finds no
		// real crossings and the INTERVAL_TIMER stall rail escalates to a
		// full restart on its own. The 4-rev gate keeps the lagging
		// average during spool-up from tripping this, and transient
		// dropouts are ridden out by blind steps.
		if (!old_routine && running) {
			if (average_interval > polling_mode_changeover + 500) {
				slow_avg_revs++;
				// High duty into an untrusted lock is the damage vector
				// (wrong-phase drive; some boards have no VDS trip): bail
				// after 2 revs instead of 4 when driving hard.
				if (slow_avg_revs >= ((duty_cycle > 500) ? 2 : 4)) {
					slow_avg_revs = 0;
					escToOpenLoop();
				}
			} else {
				slow_avg_revs = 0;
			}
		} else {
			slow_avg_revs = 0;
		}
		if (desynced) {
			slow_avg_revs = 0;
			const uint32_t zc_at_desync = zero_crosses;
			zero_crosses = 0;
			desync_happened++;
			// Same established-run gate as the stall rail (see
			// faultHandleBemfIntervalStall): interval jumps while the
			// loop is still acquiring (zc 11..100) are normal startup
			// roughness on light motors - charging them stacks holdoff
			// and ramp back-off onto honest starts until the bucket
			// latches a motor that never got going (SITL racer model
			// reproduces this under plain dshot spool). Legacy desync
			// handling below still restarts; only the episode
			// accounting is established-runs-only.
			if (zc_at_desync > 100) {
				faultDesyncEpisodeCharge(DESYNC_EPISODE_JUMP);
			}
			if ((!eepromBuffer.bi_direction && (input > 47)) || commutation_interval > 1000) {
				running = 0;
			}
			/* Always fall back to poll-ZC path after a desync event. */
			escNoteStallOrDesync(0);
			if (zero_crosses > 100) {
				average_interval = 5000;
			}
			last_duty_cycle = min_startup_duty / 2;
			if (faultDesyncRestartHoldoffActive() || escIsFault()) {
				running = 0;
				allOff();
				maskPhaseInterrupts();
			}
		}
		desync_check = 0;
		//	}
		last_average_interval = average_interval;
	}
}

void runtimeSampleBemfPreLevel(void)
{
	// Demag-late crossing detection, sampling half. In a healthy window the
	// floating phase dwells at the PRE-crossing comparator level from demag
	// end until the crossing - at the commutation rates this check covers
	// (CI > 500 ticks = 250 us+) that dwell spans many main-loop passes, so
	// a single flag set from here is reliable. In the mistimed-lock failure
	// (bench 2026-07-22: snap starts settle ~30% slow at 8-10x current) the
	// freewheel demag clamp holds the phase at the POST-crossing level for
	// the entire window, the EXTI edge fires only when demag ends, and the
	// confirm loop accepts it instantly (0 rejects/zc measured vs ~11
	// healthy) - commutation runs consistently late off demag duration, and
	// higher current makes demag longer, which is the self-locking spiral.
	// This flag is the discriminator: an accepted crossing with no observed
	// pre-level dwell is a demag-late detection (see interruptRoutine).
	// Between the accept and the next commutation the comparator holds the
	// post-crossing level of the OLD window, which never equals the current
	// `rising`, so sampling needs no window-phase gate.
	if (running && !old_routine && commutation_interval > 500) {
		if (getCompOutputLevel() == rising) {
			zc_pre_seen = 1;
		}
	}
}

void runtimeUpdateDshotIrqPriority(void)
{
#if !defined(MCU_G031) && !defined(NEED_INPUT_READY)
#	ifdef NXP
	if (dshot_telemetry && (commutation_interval > DSHOT_PRIORITY_THRESHOLD)) {
		NVIC_SetPriority(IC_DMA_IRQ_NAME, 0);
		NVIC_SetPriority(COM_TIMER_IRQ, 1);
		NVIC_SetPriority(COMP0_IRQ, 1);
		NVIC_SetPriority(COMP1_IRQ, 1);
	} else {
		NVIC_SetPriority(IC_DMA_IRQ_NAME, 1);
		NVIC_SetPriority(COM_TIMER_IRQ, 0);
		NVIC_SetPriority(COMP0_IRQ, 0);
		NVIC_SetPriority(COMP1_IRQ, 0);
	}
#	else
	if (dshot_telemetry && (commutation_interval > DSHOT_PRIORITY_THRESHOLD)) {
		NVIC_SetPriority(IC_DMA_IRQ_NAME, 0);
		NVIC_SetPriority(COM_TIMER_IRQ, 1);
		NVIC_SetPriority(COMPARATOR_IRQ, 1);
	} else {
		NVIC_SetPriority(IC_DMA_IRQ_NAME, 1);
		NVIC_SetPriority(COM_TIMER_IRQ, 0);
		NVIC_SetPriority(COMPARATOR_IRQ, 0);
	}
#	endif
#endif
}

void runtimeSendTelemetryIfNeeded(void)
{
	if (send_telemetry) {
#ifdef USE_SERIAL_TELEMETRY
		makeTelemPackage((int8_t)degrees_celsius, battery_voltage, actual_current, (uint16_t)(consumed_current >> 16), e_rpm);
		send_telem_DMA(10);
		send_telemetry = 0;
#endif
	} else if (send_esc_info_flag) {
		makeInfoPacket();
		send_telem_DMA(49);
		send_esc_info_flag = 0;
	}
}

/*
 * Transient governor, 1 kHz (main-loop context, fresh battery_voltage).
 *
 * (a) Voltage-compensated ramp: max_ramp_* are duty-per-tick, so the same
 * setting slews 1.7x the VOLTS/ms on 8S that it does on 4S. Scale the
 * working copies by GOV_RAMP_VREF_CV/Vbat (clamped 0.5x-1.5x) so a tuned
 * ramp means the same electrical transient on any pack.
 *
 * (b) BEMF-headroom ceiling: bound duty to the equilibrium duty implied by
 * the LIVE eRPM plus a fixed voltage headroom. Slip current is
 * (duty*Vbat - BEMF)/R, so capping the headroom bounds slip current and
 * demag duration without a current shunt (this board has one shunt for
 * all four ESCs - per-motor current limiting is not available). Prop
 * inertia never needs to be known: on a light prop rpm chases duty within
 * ms and the ceiling never binds; on a heavy prop duty waits for rpm, so
 * a snap becomes the fastest spool the prop can physically follow (bench
 * 2026-07-23: snap 0.20->0.55 on the 10" at 4%/ms ramp = blind-grind at
 * 100+ A; the same snap at 0.1%/ms = healthy synced 67 A spool).
 *
 * The gain (eRPM per duty*volt) is MEASURED, not taken from the eeprom KV
 * field: at steady state duty*Vbat ~= BEMF + I*R, so obs = e_rpm/(duty*V)
 * is observable every loop and load bias errs conservative (lower gain ->
 * lower ceiling). A misconfigured KV byte is exactly the failure class
 * this protects against, so it must not be an input. The estimator only
 * samples in trusted steady state (slew settled, no blind/demag/grind,
 * NOT riding the ceiling - riding it would drag the estimate down and
 * under-spool), and the ceiling only arms after GOV_CONF_ARM eligible
 * samples (~0.3 s cumulative steady running per power-up).
 */
#ifndef BRUSHED_MODE
#	define GOV_RAMP_VREF_CV 1480 /* 4S nominal: legacy ramp feel is preserved there */
#	define GOV_HEADROOM_CV 300   /* 3.0 V of slip headroom above the BEMF line */
#	define GOV_CEIL_FLOOR 350    /* ceiling never below this (nor min_startup_duty+100) */
#	define GOV_CONF_ARM 300

__attribute__((optimize("Os"))) static void runtimeTransientGovernorTick(void)
{
	// gov_slope_q10: duty units per e_rpm unit, Q10 (duty<<10/e_rpm at
	// steady state). Pack voltage folds into the slope at estimation time
	// - no live-voltage term in the ceiling. Sag/pack-swap staleness errs
	// conservative (V drop -> true equilibrium duty rises -> stale ceiling
	// is low) and the estimator re-tracks in ~30 ms of steady running.
	static uint16_t gov_slope_q10;
	static uint16_t gov_conf;
	// One volatile read each; everything below works on locals (M0: every
	// volatile re-read is a literal-pool load + ldr, and this function is
	// flash-budget critical).
	const uint32_t v = battery_voltage;
	const uint32_t erpm = e_rpm;
	const uint32_t duty = duty_cycle;
	const uint8_t closed = (running && !old_routine && zero_crosses > 150);

	// (a) voltage-compensated ramp working copies
	uint32_t scale_q8 = 256;
	if (v > 600) {
		scale_q8 = ((uint32_t)GOV_RAMP_VREF_CV << 8) / v;
		if (scale_q8 < 128) {
			scale_q8 = 128;
		}
		if (scale_q8 > 384) {
			scale_q8 = 384;
		}
	}
	uint16_t r;
	r = (uint16_t)((max_ramp_startup * scale_q8) >> 8);
	max_ramp_startup_vcomp = r ? (uint8_t)r : 1;
	r = (uint16_t)((max_ramp_low_rpm * scale_q8) >> 8);
	max_ramp_low_rpm_vcomp = r ? (uint8_t)r : 1;
	r = (uint16_t)((max_ramp_high_rpm * scale_q8) >> 8);
	max_ramp_high_rpm_vcomp = r ? (uint8_t)r : 1;

	// (b) slope estimator, steady trusted closed loop only (slew settled,
	// no blind/demag/grind, not riding the ceiling - riding it would drag
	// the estimate down and under-spool)
	if (closed && duty > 250 && last_duty_cycle == duty_cycle_setpoint && duty_cycle_setpoint < gov_duty_ceiling &&
	    zc_blind_steps == 0 && zc_demag_run == 0 && zc_grind_hold_ms == 0 && erpm > 32) {
		uint32_t obs = (duty << 10) / erpm; // erpm > 32 keeps this in uint16
		if (gov_conf == 0) {
			gov_slope_q10 = (uint16_t)obs;
		} else {
			gov_slope_q10 = (uint16_t)(gov_slope_q10 + (((int32_t)obs - gov_slope_q10) >> 5));
		}
		if (gov_conf < 1000) {
			gov_conf++;
		}
	}

	// (c) BEMF-headroom ceiling from the live eRPM: equilibrium duty via
	// the slope (multiply, no divide) plus headroom in duty units.
	// headroom = HEADROOM_CV*2000/Vbat, folded into scale_q8
	// (= VREF<<8/Vbat) to save the divide: 300*2000/1480/256 ~= 405/256.
	// The headroom bounds slip MAGNITUDE only; it cannot prevent a
	// too-fast ramp from breaking lock (bench rpmhead-snap-40: even 1 V
	// applied in 4 ms desyncs where 24 A reached gradually stays locked -
	// the cliff is dV/dt, not level; that is the learned ramp back-off's
	// job, faultDesyncEpisodeCharge).
	uint16_t ceiling = 2000;
	if (closed && gov_conf >= GOV_CONF_ARM) {
		uint32_t c = ((erpm * gov_slope_q10) >> 10) + ((scale_q8 * 405u) >> 8);
		if (c < GOV_CEIL_FLOOR) {
			c = GOV_CEIL_FLOOR;
		}
		if (c < 2000) {
			ceiling = (uint16_t)c;
		}
	}
	gov_duty_ceiling = ceiling;
}
#endif

void runtimeProcessAdcAndProtections(void)
{
	if (PROCESS_ADC_FLAG == 1) { // for adc and telemetry set adc counter at 1khz loop rate
		adcAppServiceConversion();
#ifndef BRUSHED_MODE
		runtimeTransientGovernorTick();
#endif
		if (eepromBuffer.low_voltage_cut_off == 1) {
			if (battery_voltage < (cell_count * low_cell_volt_cutoff)) {
				low_voltage_count++;
			} else {
				if (!LOW_VOLTAGE_CUTOFF) { // if set low cutoff has happened, require power cycle to reset
					low_voltage_count = 0;
				}
			}
		}
		if (eepromBuffer.low_voltage_cut_off == 2) { // absolute cut off
			if (battery_voltage < (eepromBuffer.absolute_voltage_cutoff * 50)) {
				low_voltage_count++;
			} else {
				if (!LOW_VOLTAGE_CUTOFF) {
					low_voltage_count = 0;
				}
			}
		}
		if (low_voltage_count > (10000 - (escInSineStart() * 9900))) { // 10 second wait before cut-off for low voltage
			allOff();
			maskPhaseInterrupts();
			zero_input_count = 0;
			escToFaultLvc();
		}

		PROCESS_ADC_FLAG = 0;
#ifdef USE_ADC_INPUT
		if (ADC_raw_input < 10) {
			zero_input_count++;
		} else {
			zero_input_count = 0;
		}
#endif
	}
#ifdef USE_ADC_INPUT
	signaltimeout = 0;
	ADC_smoothed_input = (((10 * ADC_smoothed_input) + ADC_raw_input) / 11);
	newinput = ADC_smoothed_input / 2;
	if (newinput > 2000) {
		newinput = 2000;
	}
#endif
}

void runtimeMotorModeTick(void)
{
	/* Once per main loop: ISR flag side-effects → named esc_state (not in 20 kHz). */
	escReconcileFromFlags();
	stuckcounter = 0;
	/* Post-desync coast: do not re-enter six-step until holdoff expires.
	 * (setInput's start branch is gated the same way - this alone cannot
	 * hold the motor off, it only kills a run that was already going.) */
	if (faultDesyncRestartHoldoffActive() || escIsFault()) {
		if (running) {
			running = 0;
			allOff();
			maskPhaseInterrupts();
		}
		// The early return skips the e_rpm update below; zero it so
		// telemetry (DroneCAN) reports the coast instead of the last
		// running rpm for up to the whole holdoff.
		e_rpm = 0;
		k_erpm = 0;
		return;
	}
	if (!escInSineStart()) {
		e_rpm = running * (600000 / e_com_time); // in tens of rpm
		k_erpm = e_rpm / 10;			 // ecom time is time for one electrical revolution in microseconds

		if (low_rpm_throttle_limit) { // some hardware doesn't need this, its on
			// by default to keep hardware / motors
			// protected but can slow down the response
			// in the very low end a little.
			duty_cycle_maximum = map(k_erpm, low_rpm_level, high_rpm_level, throttle_max_at_low_rpm,
						 throttle_max_at_high_rpm); // for more performance lower the
									    // high_rpm_level, set to a
									    // consvervative number in source.
		} else {
			duty_cycle_maximum = 2000;
		}

		if (degrees_celsius > eepromBuffer.limits.temperature) {
			duty_cycle_maximum = map(degrees_celsius, eepromBuffer.limits.temperature - 10,
						 eepromBuffer.limits.temperature + 10, throttle_max_at_high_rpm / 2, 1);
		}
		if (zero_crosses < 100 && commutation_interval > 500) {
			filter_level = ZC_FILTER_MAX;
		} else {
			filter_level = map(average_interval, 100, 500, ZC_FILTER_RUN_MIN, ZC_FILTER_MAX);
		}
		if (commutation_interval < 50) {
			filter_level = ZC_FILTER_FAST;
		}

		if (eepromBuffer.auto_advance) {
			auto_advance_level = map(duty_cycle, 100, 2000, 13, 23);
		}

		/**************** old routine*********************/
#ifdef CUSTOM_RAMP
		if (escInPollZcDrive()) {
			maskPhaseInterrupts();
			getBemfState();
			if (!zcfound) {
				if (rising) {
					if (bemfcounter > min_bemf_counts_up) {
						zcfound = 1;
						zcfoundroutine();
					}
				} else {
					if (bemfcounter > min_bemf_counts_down) {
						zcfound = 1;
						zcfoundroutine();
					}
				}
			}
		}
#endif
		faultHandleBemfIntervalStall();
	} else { // stepper sine

#ifdef GIMBAL_MODE
		step_delay = 300;
		maskPhaseInterrupts();
		allpwm();
		if (newinput > 1000) {
			desired_angle = map(newinput, 1000, 2000, 180, 360);
		} else {
			desired_angle = map(newinput, 0, 1000, 0, 180);
		}
		if (current_angle > desired_angle) {
			forward = 1;
			advanceincrement();
			delayMicros(step_delay);
			current_angle--;
		}
		if (current_angle < desired_angle) {
			forward = 0;
			advanceincrement();
			delayMicros(step_delay);
			current_angle++;
		}
#else

		if (input > 48 && escIsArmed()) {
			if (input > 48 && input < 137) { // sine wave stepper

				if (do_once_sinemode) {
					// disable commutation interrupt in case set
					DISABLE_COM_TIMER_INT();
					maskPhaseInterrupts();
					SET_DUTY_CYCLE_ALL(0);
					allpwm();
					do_once_sinemode = 0;
				}
				advanceincrement();
				step_delay = map(input, 48, 120, 7000 / eepromBuffer.motor_poles, 810 / eepromBuffer.motor_poles);
				delayMicros(step_delay);
				e_rpm = 600 / step_delay; // in hundreds so 33 e_rpm is 3300 actual erpm

			} else {
				do_once_sinemode = 1;
				advanceincrement();
				if (input > 200) {
					phase_A_position = 0;
					step_delay = 80;
				}

				delayMicros(step_delay);
				if (phase_A_position == 0) {
					escSineHandoffToOpenLoop();
					commutation_interval = 9000;
					average_interval = 9000;
					last_average_interval = average_interval;
					SET_INTERVAL_TIMER_COUNT(9000);
					zero_crosses = 20;
					step = changeover_step;
					// comStep(step);// rising bemf on a same as position 0.
					if (eepromBuffer.stall_protection) {
						last_duty_cycle = stall_protect_minimum_duty;
					}
					commutate();
					generatePwmTimerEvent();
				}
			}

		} else {
			do_once_sinemode = 1;
			if (eepromBuffer.brake_on_stop == 1) {
#	ifndef PWM_ENABLE_BRIDGE
				prop_brake_duty_cycle = eepromBuffer.drag_brake_strength * 200;
				adjusted_duty_cycle = tim1_arr - ((prop_brake_duty_cycle * tim1_arr) / 2000);
				if (adjusted_duty_cycle < 100) {
					fullBrake();
				} else {
					proportionalBrake();
					SET_DUTY_CYCLE_ALL(adjusted_duty_cycle);
					prop_brake_active = 1;
				}
#	else
				// todo add braking for PWM /enable style bridges.
#	endif
			} else if (eepromBuffer.brake_on_stop == 2) {
				comStep(2);
				SET_DUTY_CYCLE_ALL(DEAD_TIME + ((eepromBuffer.active_brake_power * tim1_arr) / 2000) * 10);
			} else {
				SET_DUTY_CYCLE_ALL(0);
				allOff();
			}
			e_rpm = 0;
		}

#endif // gimbal mode
	} // stepper/sine mode end
}
