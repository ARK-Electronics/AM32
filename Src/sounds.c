/*
 * sounds.c
 *
 *  Created on: May 13, 2020
 *      Author: Alka
 */

#include "sounds.h"
#include "common.h"
#include "eeprom.h"
#include "functions.h"
#include "peripherals.h"
#include "phaseouts.h"
#include "targets.h"

#ifndef ERASED_FLASH_BYTE
#	define ERASED_FLASH_BYTE 0xFF
#endif

uint8_t beep_volume;

void pause(uint16_t ms)
{
	SET_DUTY_CYCLE_ALL(0);
	delayMillis(ms);
	SET_DUTY_CYCLE_ALL(beep_volume); // volume of the beep, (duty cycle) don't go
					 // above 25 out of 2000
}

void setVolume(uint8_t volume)
{
	if (volume > 11) {
		volume = 11;
	}
	beep_volume = volume * 3; // volume variable from 0 - 11 equates to CCR value of 0-33
}

void setCaptureCompare()
{
	SET_DUTY_CYCLE_ALL(beep_volume); // volume of the beep, (duty cycle)
}

/*
 * @Brief 	Freq in hz, bduration in ms
 */
void playBJNote(uint16_t freq, uint16_t bduration)
{
#ifdef NXP
	uint32_t PWM_IPBUS_CLOCK_HZ = 192000000;
	SET_ACTUAL_PRESCALER_PWM(7);				// Set prescaler to 128 (max)
	SET_AUTO_RELOAD_PWM((PWM_IPBUS_CLOCK_HZ / 128) / freq); // Set PWM reload time to corresponding frequency
	SET_DUTY_CYCLE_ALL(beep_volume);			// Set beep volume (between 0 and 22, see setVolume())
#else
	uint16_t timerOne_reload;
	SET_PRESCALER_PWM(9);
	timerOne_reload = (uint16_t)(CPU_FREQUENCY_MHZ * 100000 / freq);
	SET_AUTO_RELOAD_PWM(timerOne_reload);
	SET_DUTY_CYCLE_ALL(beep_volume * timerOne_reload / TIM1_AUTORELOAD);
	delayMillis(bduration);
#endif
}

uint16_t getBlueJayNoteFrequency(uint8_t bjarrayfreq)
{
	return (uint16_t)(10000000 / ((uint32_t)bjarrayfreq * 247 + 4000));
}

void playBlueJayTune(void)
{
	uint8_t full_time_count = 0;
	uint32_t duration;
	uint16_t frequency;
	uint8_t t4, t3;
	comStep(3);

	for (int i = 4; i < 128; i += 2) {
		RELOAD_WATCHDOG_COUNTER();
		signaltimeout = 0;
		t4 = eepromBuffer.tune[i];
		t3 = eepromBuffer.tune[i + 1];
		if (t4 == 0 && t3 == 0) {
			break;
		}

		if (t4 == 255 && t3 != 0) {
			full_time_count++;

		} else if (t3 == 0) {
			duration = (uint32_t)full_time_count * 255 + t4;
			SET_DUTY_CYCLE_ALL(0);
			delayMillis((uint16_t)duration);
			full_time_count = 0;

		} else {
			uint32_t total_pulses = (uint32_t)full_time_count * 255 + t4;
			uint32_t t3_period = (uint32_t)t3 * 247 + 4000;
			duration = (total_pulses * t3_period) / 11000;

			frequency = getBlueJayNoteFrequency(t3);
			playBJNote(frequency, (uint16_t)duration);
			full_time_count = 0;
		}
		if (eepromBuffer.tune[3] > 239) {
			SET_DUTY_CYCLE_ALL(0);
			delayMillis(10 * (255 - eepromBuffer.tune[3]));
		}
	}

	allOff();
	SET_PRESCALER_PWM(0);
	SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
	signaltimeout = 0;
	RELOAD_WATCHDOG_COUNTER();
}

// equal-tempered pitches of the ARK signature arpeggio
#define ARK_NOTE_C6 1047
#define ARK_NOTE_E6 1319
#define ARK_NOTE_G6 1568

#define ARK_MORSE_UNIT_MS 50 // startup tune dot length; dash is 3 units
// arm/beacon tunes run inside the control loop with IRQs off, so they use a
// faster dot to keep the busy-wait no longer than the old ~300 ms tune
#define ARK_ARM_MORSE_UNIT_MS 40

static void playArkMorseLetter(const char *code, uint16_t freq, uint16_t unit_ms)
{
	while (*code) {
		RELOAD_WATCHDOG_COUNTER();
		playBJNote(freq, (*code == '-') ? (3 * unit_ms) : unit_ms);
		SET_DUTY_CYCLE_ALL(0); // silence between elements
		delayMillis(unit_ms);
		code++;
	}
}

// ARK signature tune: "ARK" in morse code (.- .-. -.-), each letter one
// step up a C major arpeggio so the tune rises like the stock beeps did;
// letters also walk the commutation steps the stock beeps used
static void playArkTune(void)
{
	playArkMorseLetter(".-", ARK_NOTE_C6, ARK_MORSE_UNIT_MS); // A
	delayMillis(2 * ARK_MORSE_UNIT_MS);
	comStep(5);
	playArkMorseLetter(".-.", ARK_NOTE_E6, ARK_MORSE_UNIT_MS); // R
	delayMillis(2 * ARK_MORSE_UNIT_MS);
	comStep(6);
	playArkMorseLetter("-.-", ARK_NOTE_G6, ARK_MORSE_UNIT_MS); // K
}

void playStartupTune()
{
	__disable_irq();
	comStep(3);
	if (eepromBuffer.tune[0] != ERASED_FLASH_BYTE) {
		playBlueJayTune();
	} else {
		playArkTune();
		allOff();	      // turn all channels low again
		SET_PRESCALER_PWM(0); // set prescaler back to 0.
		signaltimeout = 0;
	}

	SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
	__enable_irq();
}

void playBrushedStartupTune()
{
	__disable_irq();
	SET_AUTO_RELOAD_PWM(TIM1_AUTORELOAD);
	setCaptureCompare();
	comStep(1);	       // activate a pwm channel
	SET_PRESCALER_PWM(40); // frequency of beep
	delayMillis(300);      // duration of beep
	comStep(2);	       // activate a pwm channel
	SET_PRESCALER_PWM(30); // frequency of beep
	delayMillis(300);      // duration of beep
	comStep(3);	       // activate a pwm channel
	SET_PRESCALER_PWM(25); // frequency of beep
	delayMillis(300);      // duration of beep
	comStep(4);
	SET_PRESCALER_PWM(20); // higher again..
	delayMillis(300);
	allOff();	      // turn all channels low again
	SET_PRESCALER_PWM(0); // set prescaler back to 0.
	signaltimeout = 0;
	SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
	__enable_irq();
}

void playDuskingTune()
{
	setCaptureCompare();
	SET_AUTO_RELOAD_PWM(TIM1_AUTORELOAD);
	comStep(2);	       // activate a pwm channel
	SET_PRESCALER_PWM(60); // frequency of beep
	delayMillis(200);      // duration of beep
	SET_PRESCALER_PWM(55); // next beep is higher frequency
	delayMillis(150);
	SET_PRESCALER_PWM(50); // higher again..
	delayMillis(150);
	SET_PRESCALER_PWM(45); // frequency of beep
	delayMillis(100);      // duration of beep
	SET_PRESCALER_PWM(50); // next beep is higher frequency
	delayMillis(100);
	SET_PRESCALER_PWM(55); // higher again..
	delayMillis(100);
	SET_PRESCALER_PWM(25); // higher again..
	delayMillis(200);
	SET_PRESCALER_PWM(55); // higher again..
	delayMillis(150);
	allOff();	      // turn all channels low again
	SET_PRESCALER_PWM(0); // set prescaler back to 0.
	SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
}

// dshot beacon 4 / deferred arm beep: same morse "R" as playInputTune but
// one arpeggio step lower so the beacon is distinguishable by pitch
void playInputTune2()
{
	__disable_irq();
	RELOAD_WATCHDOG_COUNTER();
	comStep(1);
	playArkMorseLetter(".-.", ARK_NOTE_E6, ARK_ARM_MORSE_UNIT_MS); // R
	allOff();
	SET_PRESCALER_PWM(0);
	signaltimeout = 0;
	SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
	__enable_irq();
}

// signal input lock / armed: morse "R" (.-.) — "roger, signal received" —
// on the top note of the ARK startup arpeggio
void playInputTune()
{
	__disable_irq();
	RELOAD_WATCHDOG_COUNTER();
	comStep(3);
	playArkMorseLetter(".-.", ARK_NOTE_G6, ARK_ARM_MORSE_UNIT_MS); // R
	allOff();
	SET_PRESCALER_PWM(0);
	signaltimeout = 0;
	SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
	__enable_irq();
}

void playDefaultTone()
{
	SET_AUTO_RELOAD_PWM(TIM1_AUTORELOAD);
	SET_PRESCALER_PWM(50);
	setCaptureCompare();
	comStep(2);
	delayMillis(150);
	RELOAD_WATCHDOG_COUNTER();
	SET_PRESCALER_PWM(30);
	delayMillis(150);
	allOff();
	SET_PRESCALER_PWM(0);
	signaltimeout = 0;
	SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
}

void playChangedTone()
{
	SET_AUTO_RELOAD_PWM(TIM1_AUTORELOAD);
	SET_PRESCALER_PWM(40);
	setCaptureCompare();
	comStep(2);
	delayMillis(150);
	RELOAD_WATCHDOG_COUNTER();
	SET_PRESCALER_PWM(80);
	delayMillis(150);
	allOff();
	SET_PRESCALER_PWM(0);
	signaltimeout = 0;
	SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
}

void playBeaconTune3()
{
	SET_AUTO_RELOAD_PWM(TIM1_AUTORELOAD);
	__disable_irq();
	setCaptureCompare();
	for (int i = 119; i > 0; i = i - 2) {
		RELOAD_WATCHDOG_COUNTER();
		comStep(i / 20);
		SET_PRESCALER_PWM(10 + (i / 2));
		delayMillis(10);
	}
	allOff();
	SET_PRESCALER_PWM(0);
	signaltimeout = 0;
	SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
	__enable_irq();
}
