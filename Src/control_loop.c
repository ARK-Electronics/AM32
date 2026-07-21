/*
 * control_loop.c - extracted from main.c (behavior-neutral split)
 */

#include "control_loop.h"
#include "motor_runtime.h"
#include "faults.h"
#include "esc_state.h"
#include "commutation.h"
#include "bemf_zc.h"
#include "main.h"
#include "common.h"
#include "comparator.h"
#include "phaseouts.h"
#include "targets.h"
#include "IO.h"
#include "peripherals.h"
#include "functions.h"
#include "eeprom.h"
#include "dshot.h"
#include "signal.h"
#include "sounds.h"
#include "hwci_perf.h"
#include "ADC.h"
#include "kiss_telemetry.h"
#include "zc_handoff.h"
#ifdef USE_LED_STRIP
#	ifndef NXP
#		include "WS2812.h"
#	endif
#endif
#ifdef USE_CRSF_INPUT
#	include "crsf.h"
#endif
#if DRONECAN_SUPPORT
#	include "DroneCAN/DroneCAN.h"
#endif

int32_t doPidCalculations(struct fastPID *pidnow, int actual, int target)
{
	pidnow->error = actual - target;
	pidnow->integral = pidnow->integral + pidnow->error * pidnow->Ki;
	if (pidnow->integral > pidnow->integral_limit) {
		pidnow->integral = pidnow->integral_limit;
	}
	if (pidnow->integral < -pidnow->integral_limit) {
		pidnow->integral = -pidnow->integral_limit;
	}

	pidnow->derivative = pidnow->Kd * (pidnow->error - pidnow->last_error);
	pidnow->last_error = pidnow->error;

	pidnow->pid_output = pidnow->error * pidnow->Kp + pidnow->integral + pidnow->derivative;

	if (pidnow->pid_output > pidnow->output_limit) {
		pidnow->pid_output = pidnow->output_limit;
	}
	if (pidnow->pid_output < -pidnow->output_limit) {
		pidnow->pid_output = -pidnow->output_limit;
	}
	return pidnow->pid_output;
}

uint16_t getSmoothedCurrent()
{
	total = total - readings[readIndex];
	readings[readIndex] = ADC_raw_current;
	total = total + readings[readIndex];
	readIndex = readIndex + 1;
	if (readIndex >= numReadings) {
		readIndex = 0;
	}
	smoothedcurrent = total / numReadings;
	return smoothedcurrent;
}

void setInput()
{
	if (eepromBuffer.bi_direction) {
		if (dshot == 0) {
			if (eepromBuffer.rc_car_reverse) {
				if (newinput > (1000 + (servo_dead_band << 1))) {
					if (forward == eepromBuffer.dir_reversed) {
						adjusted_input = 0;
						//               if (running) {
						prop_brake_active = 1;
						if (return_to_center) {
							forward = 1 - eepromBuffer.dir_reversed;
							prop_brake_active = 0;
							return_to_center = 0;
						}
					}
					if (prop_brake_active == 0) {
						return_to_center = 0;
						adjusted_input = map(newinput, 1000 + (servo_dead_band << 1), 2000, 47, 2047);
					}
				}
				if (newinput < (1000 - (servo_dead_band << 1))) {
					if (forward == (1 - eepromBuffer.dir_reversed)) {
						adjusted_input = 0;
						prop_brake_active = 1;
						if (return_to_center) {
							forward = eepromBuffer.dir_reversed;
							prop_brake_active = 0;
							return_to_center = 0;
						}
					}
					if (prop_brake_active == 0) {
						return_to_center = 0;
						adjusted_input = map(newinput, 0, 1000 - (servo_dead_band << 1), 2047, 47);
					}
				}
				if (newinput >= (1000 - (servo_dead_band << 1)) && newinput <= (1000 + (servo_dead_band << 1))) {
					adjusted_input = 0;
					if (prop_brake_active) {
						prop_brake_active = 0;
						return_to_center = 1;
					}
				}
			} else {
				if (newinput > (1000 + (servo_dead_band << 1))) {
					if (forward == eepromBuffer.dir_reversed) {
						if (((commutation_interval > reverse_speed_threshold) && (duty_cycle < 200)) ||
						    escInSineStart()) {
							forward = 1 - eepromBuffer.dir_reversed;
							zero_crosses = 0;
							old_routine = 1;
							zcHandoffReset();
							maskPhaseInterrupts();
							brushed_direction_set = 0;
						} else {
							newinput = 1000;
						}
					}
					adjusted_input = map(newinput, 1000 + (servo_dead_band << 1), 2000, 47, 2047);
				}
				if (newinput < (1000 - (servo_dead_band << 1))) {
					if (forward == (1 - eepromBuffer.dir_reversed)) {
						if (((commutation_interval > reverse_speed_threshold) && (duty_cycle < 200)) ||
						    escInSineStart()) {
							zero_crosses = 0;
							old_routine = 1;
							zcHandoffReset();
							forward = eepromBuffer.dir_reversed;
							maskPhaseInterrupts();
							brushed_direction_set = 0;
						} else {
							newinput = 1000;
						}
					}
					adjusted_input = map(newinput, 0, 1000 - (servo_dead_band << 1), 2047, 47);
				}

				if (newinput >= (1000 - (servo_dead_band << 1)) && newinput <= (1000 + (servo_dead_band << 1))) {
					adjusted_input = 0;
					brushed_direction_set = 0;
				}
			}
		}
		if (dshot) {
			if (eepromBuffer.rc_car_reverse) {
				if (newinput > 1047) {
					if (forward == eepromBuffer.dir_reversed) {
						adjusted_input = 0;
						prop_brake_active = 1;
						if (return_to_center) {
							forward = 1 - eepromBuffer.dir_reversed;
							prop_brake_active = 0;
							return_to_center = 0;
						}
					}
					if (prop_brake_active == 0) {
						return_to_center = 0;
						adjusted_input = ((newinput - 1048) * 2 + 47) - reversing_dead_band;
					}
				}
				if (newinput <= 1047 && newinput > 47) {
					if (forward == (1 - eepromBuffer.dir_reversed)) {
						adjusted_input = 0;
						prop_brake_active = 1;
						if (return_to_center) {
							forward = eepromBuffer.dir_reversed;
							prop_brake_active = 0;
							return_to_center = 0;
						}
					}
					if (prop_brake_active == 0) {
						return_to_center = 0;
						adjusted_input = ((newinput - 48) * 2 + 47) - reversing_dead_band;
					}
				}
				if (newinput < 48) {
					adjusted_input = 0;
					if (prop_brake_active) {
						prop_brake_active = 0;
						return_to_center = 1;
					}
				}
			} else {
				if (newinput > 1047) {
					if (forward == eepromBuffer.dir_reversed) {
						if (((commutation_interval > reverse_speed_threshold) && (duty_cycle < 200)) ||
						    escInSineStart()) {
							forward = 1 - eepromBuffer.dir_reversed;
							zero_crosses = 0;
							old_routine = 1;
							zcHandoffReset();
							maskPhaseInterrupts();
							brushed_direction_set = 0;
						} else {
							newinput = 0;
						}
					}
					adjusted_input = ((newinput - 1048) * 2 + 47) - reversing_dead_band;
				}
				if (newinput <= 1047 && newinput > 47) {
					if (forward == (1 - eepromBuffer.dir_reversed)) {
						if (((commutation_interval > reverse_speed_threshold) && (duty_cycle < 200)) ||
						    escInSineStart()) {
							zero_crosses = 0;
							old_routine = 1;
							zcHandoffReset();
							forward = eepromBuffer.dir_reversed;
							maskPhaseInterrupts();
							brushed_direction_set = 0;
						} else {
							newinput = 0;
						}
					}
					adjusted_input = ((newinput - 48) * 2 + 47) - reversing_dead_band;
				}
				if (newinput < 48) {
					adjusted_input = 0;
					brushed_direction_set = 0;
				}
			}
		}
	} else {
		adjusted_input = newinput;
	}
#ifndef BRUSHED_MODE
	if (faultHandleStuckRotorIfNeeded()) {
		/* drive cut and latched; skip normal throttle map */
	} else {
#	ifdef FIXED_DUTY_MODE
		input = FIXED_DUTY_MODE_POWER * 20 + 47;
#	else
		if (eepromBuffer.use_sine_start) {
			if (adjusted_input < 30) { // dead band ?
				input = 0;
			}
			if (adjusted_input > 30 && adjusted_input < (eepromBuffer.sine_mode_changeover_thottle_level * 20)) {
				input = map(adjusted_input, 30, (eepromBuffer.sine_mode_changeover_thottle_level * 20), 47, 160);
			}
			if (adjusted_input >= (eepromBuffer.sine_mode_changeover_thottle_level * 20)) {
				input = map(adjusted_input, (eepromBuffer.sine_mode_changeover_thottle_level * 20), 2047, 160, 2047);
			}
		} else {
			if (use_speed_control_loop) {
				if (drive_by_rpm) {
					target_e_com_time =
						60000000 /
						map(adjusted_input, 47, 2047, MINIMUM_RPM_SPEED_CONTROL, MAXIMUM_RPM_SPEED_CONTROL) /
						(eepromBuffer.motor_poles / 2);
					if (adjusted_input < 47) { // dead band ?
						input = 0;
						speedPid.error = 0;
						input_override = 0;
					} else {
						input = (uint16_t)(input_override / 10000); // speed control pid override
						if (input > 2047) {
							input = 2047;
						}
						if (input < 48) {
							input = 48;
						}
					}
				} else {
					input = (uint16_t)(input_override / 10000); // speed control pid override
					if (input > 2047) {
						input = 2047;
					}
					if (input < 48) {
						input = 48;
					}
				}
			} else {
				input = adjusted_input;
			}
		}
#	endif
	}
#endif
#ifndef BRUSHED_MODE
	if (escMaySixStepThrottle()) {
		if (input >= 47 + (80 * eepromBuffer.use_sine_start)) {
			if (!escIsDriving()) {
				allOff();
				if (!old_routine) {
					startMotor();
				}
				escEnterRunningOpenLoop();
				last_duty_cycle = min_startup_duty;
			}

			// straight line from (in_min, out_min) to (2047, 2000) using a
			// startup computed Q16 slope, avoids calling map() at input rate
			if (eepromBuffer.use_sine_start) {
				duty_cycle_setpoint =
					input >= 2047  ? 2000
					: input <= 137 ? minimum_duty_cycle + 40
						       : minimum_duty_cycle + 40 +
								 (uint16_t)(((uint32_t)(input - 137) * sine_throttle_duty_slope_q16) >> 16);
			} else {
				duty_cycle_setpoint =
					input >= 2047 ? 2000
					: input <= 47
						? minimum_duty_cycle
						: minimum_duty_cycle + (uint16_t)(((uint32_t)(input - 47) * throttle_duty_slope_q16) >> 16);
			}

			if (!eepromBuffer.rc_car_reverse) {
				prop_brake_active = 0;
			}
		}

		if (input < 47 + (80 * eepromBuffer.use_sine_start)) {
			if (play_tone_flag != 0) {
				switch (play_tone_flag) {
					case 1:
						playDefaultTone();
						break;
					case 2:
						playChangedTone();
						break;
					case 3:
						playBeaconTune3();
						break;
					case 4:
						playInputTune2();
						break;
					case 5:
						playDefaultTone();
						break;
				}
				play_tone_flag = 0;
			}

			if (!eepromBuffer.comp_pwm) {
				duty_cycle_setpoint = 0;
				if (!escIsDriving()) {
					old_routine = 1;
					zero_crosses = 0;
					zcHandoffReset();
					if (eepromBuffer.brake_on_stop) {
						fullBrake();
					} else {
						if (!prop_brake_active) {
							allOff();
						}
					}
				}
				if (eepromBuffer.rc_car_reverse && prop_brake_active) {
#	ifndef PWM_ENABLE_BRIDGE

					if (dshot == 0)
						prop_brake_duty_cycle = (getAbsDif(1000, newinput) + 1000);
					if (dshot) {
						if (newinput <= 1047 && newinput > 47)
							prop_brake_duty_cycle = ((newinput - 48) * 2 + 47) - reversing_dead_band;
						if (newinput > 1047)
							prop_brake_duty_cycle = ((newinput - 1048) * 2 + 47) - reversing_dead_band;
					}
					if (prop_brake_duty_cycle >= (1999)) {
						fullBrake();
					} else {
						proportionalBrake();
					}
#	endif
				}
			} else {
				if (!escIsDriving()) {
					old_routine = 1;
					zero_crosses = 0;
					zcHandoffReset();
					bad_count = 0;
					if (eepromBuffer.brake_on_stop > 0) {
						if (!eepromBuffer.use_sine_start) {
#	ifndef PWM_ENABLE_BRIDGE
							if (eepromBuffer.brake_on_stop == 1) {
								prop_brake_duty_cycle = eepromBuffer.drag_brake_strength * 200;
								if (prop_brake_duty_cycle >= (1999)) {
									fullBrake();
								} else {
									proportionalBrake();
									prop_brake_active = 1;
								}
							}
#	else
							// todo add proportional braking for pwm/enable style bridge.
#	endif
						}
					} else {
						allOff();
					}
					duty_cycle_setpoint = 0;
				}

				phase_A_position = ((step - 1) * 60) + enter_sine_angle;
				if (phase_A_position > 359) {
					phase_A_position -= 360;
				}
				phase_B_position = phase_A_position + 119;
				if (phase_B_position > 359) {
					phase_B_position -= 360;
				}
				phase_C_position = phase_A_position + 239;
				if (phase_C_position > 359) {
					phase_C_position -= 360;
				}

				if (eepromBuffer.use_sine_start == 1) {
					escToSineStart();
				}
				duty_cycle_setpoint = 0;
			}
		}
		if (!prop_brake_active) {
			if (input >= 47 && (zero_crosses < (uint32_t)(30 >> eepromBuffer.stall_protection))) {
				if (duty_cycle_setpoint < min_startup_duty) {
					duty_cycle_setpoint = min_startup_duty;
				}
				if (duty_cycle_setpoint > startup_max_duty_cycle) {
					duty_cycle_setpoint = startup_max_duty_cycle;
				}
			}

			if (duty_cycle_setpoint > duty_cycle_maximum) {
				duty_cycle_setpoint = duty_cycle_maximum;
			}
			if (use_current_limit) {
				if (duty_cycle_setpoint > use_current_limit_adjust) {
					duty_cycle_setpoint = use_current_limit_adjust;
				}
			}

			if (stall_protection_adjust > 0 && input > 47) {
				duty_cycle_setpoint = duty_cycle_setpoint + (uint16_t)(stall_protection_adjust / 10000);
			}
		}
	}
#endif
}

RAM_FUNC void tenKhzRoutine()
{ // 20khz as of 2.00 to be renamed
	HWCI_PERF_CTRL_ENTER();
	/*
     * Do not call escReconcileFromFlags() here: it lives in flash and forces
     * a long-call veneer into every 20 kHz tick. Policy predicates below are
     * flag-backed (see esc_state.h); full enum reconcile runs once per main
     * loop in runtimeMotorModeTick().
     */
	duty_cycle = duty_cycle_setpoint;
	tenkhzcounter++;
	ledcounter++;
	ramp_count++;
	one_khz_loop_counter++;
	if (!escIsArmed()) {
		if (cell_count == 0) {
			if (inputSet) {
				if (adjusted_input == 0) {
					armed_timeout_count++;
					if (armed_timeout_count > LOOP_FREQUENCY_HZ) { // one second
						if (zero_input_count > 30) {
							escToArmedIdle();
#ifdef USE_LED_STRIP
							//	send_LED_RGB(0,0,0);
							delayMicros(1000);
							send_LED_RGB(0, 255, 0);
#endif
#ifdef USE_RGB_LED
							setIndividualRGBLed(0, 1, 0);
#endif
							if ((cell_count == 0) && eepromBuffer.low_voltage_cut_off == 1) {
								cell_count = battery_voltage / 370;
								for (int i = 0; i < cell_count; i++) {
									playInputTune();
									delayMillis(100);
									RELOAD_WATCHDOG_COUNTER();
								}
							} else {
#ifdef MCU_AT415
								play_tone_flag = 4;
#else
								playInputTune();
#endif
							}
							if (!servoPwm && !dshot) {
								eepromBuffer.rc_car_reverse = 0;
							}
						} else {
							inputSet = 0;
							armed_timeout_count = 0;
							escToDisarmed();
						}
					}
				} else {
					armed_timeout_count = 0;
				}
			}
		}
	}

	if (eepromBuffer.telemetry_on_interval) {
		telem_ms_count++;
		if (telem_ms_count > ((telemetry_interval_ms - 1 + eepromBuffer.telemetry_on_interval) * 20)) {
			// telemetry_on_interval = 1 is a boolean, but it can also be 2 or more to indicate an identifier
			// by making the interval just slightly different with an unique identifier, we can guarantee that many ESCs can communicate on just one signal
			// there will be some collisions but not as many as if two ESCs always tried to talk at once.
			send_telemetry = 1;
			telem_ms_count = 0;
		}
	}

#ifndef BRUSHED_MODE

	if (!escInSineStart()) {
#	ifndef CUSTOM_RAMP
		if (escInPollZcDrive()) {
			//				send_LED_RGB(255, 0, 0);
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
#	endif
		if (one_khz_loop_counter > PID_LOOP_DIVIDER) { // 1khz PID loop
			PROCESS_ADC_FLAG = 1;		       // set flag to do new adc read at lower priority
			one_khz_loop_counter = 0;
			if (use_current_limit && escIsDriving()) {
				use_current_limit_adjust -=
					(int16_t)(doPidCalculations(&currentPid, actual_current, eepromBuffer.limits.current * 2 * 100) /
						  10000);
				if (use_current_limit_adjust < minimum_duty_cycle) {
					use_current_limit_adjust = minimum_duty_cycle;
				}
				if (use_current_limit_adjust > 2000) {
					use_current_limit_adjust = 2000;
				}
			}
			if (eepromBuffer.stall_protection && escIsDriving()) { // this boosts throttle as the rpm gets lower, for crawlers
				// and rc cars only, do not use for multirotors.
				stall_protection_adjust +=
					(doPidCalculations(&stallPid, commutation_interval, stall_protect_target_interval));
				if (stall_protection_adjust > 150 * 10000) {
					stall_protection_adjust = 150 * 10000;
				}
				if (stall_protection_adjust <= 0) {
					stall_protection_adjust = 0;
				}
			}
			if (use_speed_control_loop && escIsDriving()) {
				input_override += doPidCalculations(&speedPid, e_com_time, target_e_com_time);
				if (input_override > 2047 * 10000) {
					input_override = 2047 * 10000;
				}
				if (input_override < 0) {
					input_override = 0;
				}
				if (zero_crosses < 100) {
					speedPid.integral = 0;
				}
			}
		}
		if (ramp_count > ramp_divider) {
			ramp_count = 0;
#	ifdef VOLTAGE_BASED_RAMP
			uint16_t voltage_based_max_change = map(battery_voltage, 800, 2200, 10, 1);
			if (average_interval > 200) {
				max_duty_cycle_change = voltage_based_max_change;
			} else {
				max_duty_cycle_change = voltage_based_max_change * 3;
			}
#	else
			if (zero_crosses < 150 || last_duty_cycle < 150) {
				max_duty_cycle_change = max_ramp_startup;
			} else {
				if (average_interval > 500) {
					max_duty_cycle_change = max_ramp_low_rpm;
				} else {
					max_duty_cycle_change = max_ramp_high_rpm;
				}
			}

#	endif
#	ifdef CUSTOM_RAMP
			//         max_duty_cycle_change = eepromBuffer[30];
#	endif
			if ((duty_cycle - last_duty_cycle) > max_duty_cycle_change) {
				duty_cycle = last_duty_cycle + max_duty_cycle_change;
			}
			if ((last_duty_cycle - duty_cycle) > max_duty_cycle_change) {
				duty_cycle = last_duty_cycle - max_duty_cycle_change;
			}
		} else {
			duty_cycle = last_duty_cycle;
		}

		/* Inside !escInSineStart(): escIsDriving() ≡ running. */
		if (escIsDriving() && input > 47) {
			if (eepromBuffer.variable_pwm) {}
			adjusted_duty_cycle = (((uint32_t)duty_cycle * pwm_to_arr_scale_q16) >> 16) + 1;

		} else {
			if (escInBrake() || prop_brake_active) {
				adjusted_duty_cycle = tim1_arr - (((uint32_t)prop_brake_duty_cycle * pwm_to_arr_scale_q16) >> 16);
			} else {
				if ((eepromBuffer.brake_on_stop == 2) && escIsArmed()) { // require arming for active brake
					comStep(2);
					adjusted_duty_cycle =
						DEAD_TIME + (((uint32_t)eepromBuffer.active_brake_power * pwm_to_arr_scale_q16) >> 16) * 10;
				} else {
					adjusted_duty_cycle = (((uint32_t)duty_cycle * pwm_to_arr_scale_q16) >> 16);
				}
			}
		}
		last_duty_cycle = duty_cycle;
		SET_AUTO_RELOAD_PWM(tim1_arr);
		SET_DUTY_CYCLE_ALL(adjusted_duty_cycle);
	}
#endif // ndef brushed_mode
	faultSignalTimeoutTick();
	HWCI_PERF_CTRL_EXIT();
}

void processDshot()
{
	if (compute_dshot_flag == 1) {
		computeDshotDMA();
		compute_dshot_flag = 0;
	}
	if (compute_dshot_flag == 2) {
		if (e_com_time > 65535) { // beyond dshot range
			make_dshot_package(65535);
		} else {
			make_dshot_package(e_com_time);
		}
		compute_dshot_flag = 0;
		return;
	}
	setInput();
}
