/*
 * commutation.c - extracted from main.c (behavior-neutral split)
 */

#include "commutation.h"
#include "motor_runtime.h"
#include "main.h"
#include "common.h"
#include "comparator.h"
#include "phaseouts.h"
#include "targets.h"
#include "IO.h"
#include "peripherals.h"
#include "functions.h"
#include "eeprom.h"
#include "zc_handoff.h"

RAM_FUNC void getBemfState()
{
	uint8_t current_state = 0;
#if defined(MCU_F031) || defined(MCU_G031)
	if (step == 1 || step == 4) {
		current_state = PHASE_C_EXTI_PORT->IDR & PHASE_C_EXTI_PIN;
	}
	if (step == 2 || step == 5) { //        in phase two or 5 read from phase A Pf1
		current_state = PHASE_A_EXTI_PORT->IDR & PHASE_A_EXTI_PIN;
	}
	if (step == 3 || step == 6) { // phase B pf0
		current_state = PHASE_B_EXTI_PORT->IDR & PHASE_B_EXTI_PIN;
	}
#else
	//Get current comparator output level
	current_state = !getCompOutputLevel(); // polarity reversed
#endif
	if (rising) {
		if (current_state) {
			bemfcounter++;
		} else {
			bad_count++;
			if (bad_count > bad_count_threshold) {
				bemfcounter = 0;
			}
		}
	} else {
		if (!current_state) {
			bemfcounter++;
		} else {
			bad_count++;
			if (bad_count > bad_count_threshold) {
				bemfcounter = 0;
			}
		}
	}
}

RAM_FUNC void commutate()
{
	if (forward == 1) {
		step++;
		if (step > 6) {
			step = 1;
			desync_check = 1;
		}
		rising = step % 2;
	} else {
		step--;
		if (step < 1) {
			step = 6;
			desync_check = 1;
		}
		rising = !(step % 2);
	}
#ifdef INVERTED_EXTI
	rising = !rising;
#endif
	__disable_irq(); // don't let dshot interrupt
	if (!prop_brake_active) {
		comStep(step);
	}
	__enable_irq();
	changeCompInput();
#ifndef NO_POLLING_START
	/* Quality-based exit from closed-loop (replaces sole CI vs T+500 test). */
	if (!old_routine && zcHandoffShouldExitClosedLoop(average_interval)) {
		old_routine = 1;
		zcHandoffReset();
	}
#endif
	bemfcounter = 0;
	zcfound = 0;
	commutation_intervals[step - 1] = commutation_interval; // just used to calulate average

#ifdef USE_PULSE_OUT
	if (step == 1 || step == 4) {
		WRITE_REG(RPM_PULSE_PORT->ODR, READ_REG(RPM_PULSE_PORT->ODR) ^ RPM_PULSE_PIN);
	}
#endif
}

void advanceincrement()
{
	if (!forward) {
		phase_A_position++;
		if (phase_A_position > 359) {
			phase_A_position = 0;
		}
		phase_B_position++;
		if (phase_B_position > 359) {
			phase_B_position = 0;
		}
		phase_C_position++;
		if (phase_C_position > 359) {
			phase_C_position = 0;
		}
	} else {
		phase_A_position--;
		if (phase_A_position < 0) {
			phase_A_position = 359;
		}
		phase_B_position--;
		if (phase_B_position < 0) {
			phase_B_position = 359;
		}
		phase_C_position--;
		if (phase_C_position < 0) {
			phase_C_position = 359;
		}
	}
#ifdef GIMBAL_MODE
	setPWMCompare1(((2 * pwmSin[phase_A_position]) + gate_drive_offset) * TIMER1_MAX_ARR / 2000);
	setPWMCompare2(((2 * pwmSin[phase_B_position]) + gate_drive_offset) * TIMER1_MAX_ARR / 2000);
	setPWMCompare3(((2 * pwmSin[phase_C_position]) + gate_drive_offset) * TIMER1_MAX_ARR / 2000);
#else
	setPWMCompare1((((2 * pwmSin[phase_A_position] / SINE_DIVIDER) + gate_drive_offset) * TIMER1_MAX_ARR / 2000) *
		       eepromBuffer.sine_mode_power / 10);
	setPWMCompare2((((2 * pwmSin[phase_B_position] / SINE_DIVIDER) + gate_drive_offset) * TIMER1_MAX_ARR / 2000) *
		       eepromBuffer.sine_mode_power / 10);
	setPWMCompare3((((2 * pwmSin[phase_C_position] / SINE_DIVIDER) + gate_drive_offset) * TIMER1_MAX_ARR / 2000) *
		       eepromBuffer.sine_mode_power / 10);
#endif
}

void zcfoundroutine()
{ // only used in polling mode, blocking routine.
	thiszctime = INTERVAL_TIMER_COUNT;
	SET_INTERVAL_TIMER_COUNT(0);
	commutation_interval = (thiszctime + (3 * commutation_interval)) / 4;
	advance = (temp_advance * commutation_interval) >> 6; //   7.5 degree increments
	waitTime = commutation_interval / 2 - advance;
	while ((INTERVAL_TIMER_COUNT) < (waitTime)) {
		if (zero_crosses < 5) {
			break;
		}
	}
#ifdef MCU_GDE23
	TIMER_CAR(COM_TIMER) = waitTime;
#endif
#ifdef STMICRO
	COM_TIMER->ARR = waitTime;
#endif
#ifdef MCU_AT32
	COM_TIMER->pr = waitTime;
#endif
#ifdef NXP
	//	COM_TIMER->MSR[0] = waitTime;
	COM_TIMER->MR[0] = waitTime;
#endif

	commutate();
	bemfcounter = 0;
	bad_count = 0;

	zero_crosses++;
#ifdef NO_POLLING_START // changes to interrupt mode after 2 zero crosses, does not re-enter
	if (zero_crosses > 2) {
		old_routine = 0;
		enableCompInterrupts(); // enable interrupt
	}
#else
	if (eepromBuffer.stall_protection || eepromBuffer.rc_car_reverse) {
		if (zero_crosses >= 20 && commutation_interval <= 2000) {
			old_routine = 0;
			enableCompInterrupts(); // enable interrupt
		}
	} else {
		/* Quality handoff: stable poll intervals (and/or fast CI legacy path). */
		zcHandoffNotePollInterval(commutation_interval);
		if (zcHandoffShouldEnterClosedLoop(commutation_interval)) {
			old_routine = 0;
			enableCompInterrupts(); // enable interrupt
		}
	}
#endif
}
