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
	average_interval = e_com_time / 3;
	if (desync_check && zero_crosses > 10) {
		if ((getAbsDif(last_average_interval, average_interval) > average_interval >> 1) &&
		    (average_interval < 2000)) { // throttle resitricted before zc 20.
			zero_crosses = 0;
			desync_happened++;
			if ((!eepromBuffer.bi_direction && (input > 47)) || commutation_interval > 1000) {
				running = 0;
			}
			/* Always fall back to poll-ZC path after a desync event. */
			escNoteStallOrDesync(0);
			if (zero_crosses > 100) {
				average_interval = 5000;
			}
			last_duty_cycle = min_startup_duty / 2;
		}
		desync_check = 0;
		//	}
		last_average_interval = average_interval;
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

void runtimeProcessAdcAndProtections(void)
{
	if (PROCESS_ADC_FLAG == 1) { // for adc and telemetry set adc counter at 1khz loop rate
		adcAppServiceConversion();
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
