
/* AM32- multi-purpose brushless controller firmware for the stm32f051 */

//===========================================================================
//=============================== Changelog =================================
//===========================================================================
/*
 * 1.54 Changelog;
 * --Added firmware name to targets and firmware version to main
 * --added two more dshot to beacons 1-3 currently working
 * --added KV option to firmware, low rpm power protection is based on KV
 * --start power now controls minimum idle power as well as startup strength.
 * --change default timing to 22.5
 * --Lowered default minimum idle setting to 1.5 percent duty cycle, slider
range from 1-2.
 * --Added dshot commands to save settings and reset ESC.
 *
 *1.56 Changelog.
 * -- added check to stall protection to wait until after 40 zero crosses to fix
high startup throttle hiccup.
 * -- added TIMER 1 update interrupt and PWM changes are done once per pwm
period
 * -- reduce commutation interval averaging length
 * -- reduce false positive filter level to 2 and eliminate threshold where
filter is stopped.
 * -- disable interrupt before sounds
 * -- disable TIM1 interrupt during stepper sinusoidal mode
 * -- add 28us delay for dshot300
 * -- report 0 rpm until the first 10 successful steps.
 * -- move serial ADC telemetry calculations and desync check to 10Khz
interrupt.
 *
 * 1.57
 * -- remove spurious commutations and rpm data at startup by polling for longer
interval on startup
 *
 * 1.58
 * -- move signal timeout to 10khz routine and set armed timeout to one quarter
second 2500 / 10000
 * 1.59
 * -- moved comp order definitions to target.h
 * -- fixed update version number if older than new version
 * -- cleanup, moved all input and output to IO.c
 * -- moved comparator functions to comparator.c
 * -- removed ALOT of useless variables
 * -- added siskin target
 * -- moved pwm changes to 10khz routine
 * -- moved basic functions to functions.c
 * -- moved peripherals setup to periherals.c
 * -- added crawler mode settings
 *
 * 1.60
 * -- added sine mode hysteresis
 * -- increased power in stall protection and lowered start rpm for crawlers
 * -- removed onehot125 from crawler mode
 * -- reduced maximum startup power from 400 to 350
 * -- change minimum duty cycle to DEAD_TIME
 * -- version and name moved to permanent spot in FLASH memory, thanks mikeller
 *
 * 1.61
 * -- moved duty cycle calculation to 10khz and added max change option.
 * -- decreased maximum interval change to 25%
 * -- reduce wait time on fast acceleration (fast_accel)
 * -- added check in interrupt for early zero cross
 *
 * 1.62
 * --moved control to 10khz loop
 * --changed condition for low rpm filter for duty cycle from || to &&
 * --introduced max deceleration and set it to 20ms to go from 100 to 0
 * --added configurable servo throttle ranges
 *
 *
 *1.63
 *-- increase time for zero cross error detection below 250us commutation
interval
 *-- increase max change a low rpm x10
 *-- set low limit of throttle ramp to a lower point and increase upper range
 *-- change desync event from full restart to just lower throttle.

 *1.64
 * --added startup check for continuous high signal, reboot to enter bootloader.
 *-- added brake on stop from eeprom
 *-- added stall protection from eeprom
 *-- added motor pole divider for sinusoidal and low rpm power protection
 *-- fixed dshot commands, added confirmation beeps and removed blocking
behavior
 *--
 *1.65
 *-- Added 32 millisecond telemetry output
 *-- added low voltage cutoff , divider value and cutoff voltage needs to be
added to eeprom
 *-- added beep to indicate cell count if low voltage active
 *-- added current reading on pa3 , conversion factor needs to be added to
eeprom
 *-- fixed servo input capture to only read positive pulse to handle higher
refresh rates.
 *-- disabled oneshot 125.
 *-- extended servo range to match full output range of receivers
 *-- added RC CAR style reverse, proportional brake on first reverse , double
tap to change direction
 *-- added brushed motor control mode
 *-- added settings to EEPROM version 1
 *-- add gimbal control option.
 *--
 *1.66
 *-- move idwg init to after input tune
 *-- remove reset after save command -- dshot
 *-- added wraith32 target
 *-- added average pulse check for signal detection
 *--
 *1.67
 *-- Rework file structure for multiple MCU support
 *-- Add g071 mcu
 *--
 *1.68
 *--increased allowed average pulse length to avoid double startup
 *1.69
 *--removed line re-enabling comparator after disabling.
 *1.70 fix dshot for Kiss FC
 *1.71 fix dshot for Ardupilot / Px4 FC
 *1.72 Fix telemetry output and add 1 second arming.
 *1.73 Fix false arming if no signal. Remove low rpm throttle protection below
300kv *1.74 Add Sine Mode range and drake brake strength adjustment *1.75
Disable brake on stop for PWM_ENABLE_BRIDGE Removed automatic brake on stop on
neutral for RC car proportional brake. Adjust sine speed and stall protection
speed to more closely match makefile fixes from Cruwaller Removed gd32 build,
until firmware is functional *1.76 Adjust g071 PWM frequency, and startup power
to be same frequency as f051. Reduce number of polling back emf checks for g071
 *1.77 increase PWM frequency range to 8-48khz
 *1.78 Fix bluejay tunes frequency and speed.
           Fix g071 Dead time
           Increment eeprom version
 *1.79 Add stick throttle calibration routine
           Add variable for telemetry interval
 *1.80 -Enable Comparator blanking for g071 on timer 1 channel 4
           -add hardware group F for Iflight Blitz
           -adjust parameters for pwm frequency
           -add sine mode power variable and eeprom setting
           -fix telemetry rpm during sine mode
           -fix sounds for extended pwm range
           -Add adjustable braking strength when driving
 *1.81 -Add current limiting PID loop
           -fix current sense scale
           -Increase brake power on maximum reverse ( car mode only)
           -Add HK and Blpwr targets
           -Change low kv motor throttle limit
           -add reverse speed threshold changeover based on motor kv
           -doubled filter length for motors under 900kv
*1.82  -Add speed control pid loop.
*1.83  -Add stall protection pid loop.
           -Improve sine mode transition.
           -decrease speed step re-entering sine mode
           -added fixed duty cycle and speed mode build option
           -added rpm_controlled by input signal ( to be added to config tool )
*1.84  -Change PID value to int for faster calculations
           -Enable two channel brushed motor control for dual motors
           -Add current limit max duty cycle
*1.85  -fix current limit not allowing full rpm on g071 or low pwm frequency
                -remove unused brake on stop conditional
*1.86  - create do-once in sine mode instead of setting pwm mode each time.
*1.87  - fix fixed mode max rpm limits
*1.88  - Fix stutter on sine mode re-entry due to position reset
*1.89  - Fix drive by rpm mode scaling.
           - Fix dshot px4 timings
*1.90  - Disable comp interrupts for brushed mode
           - Re-enter polling mode after prop strike or desync
           - add G071 "N" variant
           - add preliminary Extended Dshot
*1.91  - Reset average interval time on desync only after 100 zero crosses
*1.92  - Move g071 comparator blanking to TIM1 OC5
           - Increase ADC read frequency and current sense filtering
           - Add addressable LED strip for G071 targets
*1.93  - Optimization for build process
       - Add firmware file name to each target hex file
       -fix extended telemetry not activating dshot600
       -fix low voltage cuttoff timeout
*1.94  - Add selectable input types
*1.95  - reduce timeout to 0.5 seconds when armed
*1.96  - Improved erpm accuracy dshot and serial telemetry, thanks Dj-Uran
             - Fix PID loop integral.
                 - add overcurrent low voltage cuttoff to brushed mode.
*1.97    - enable input pullup
*1.98    - Dshot erpm rounding compensation.
*1.99    - Add max duty cycle change to individual targets ( will later become
an settings option)
                 - Fix dshot telemetry delay f4 and e230 mcu
*2.00    - Cleanup of target structure
*2.01    - Increase 10khztimer to 20khz, increase max duty cycle change.
*2.02	 - Increase startup power for inverted output targets.
*2.03    - Move chime from dshot direction change commands to save command.
*2.04    - Fix current protection, max duty cycle not increasing
                 - Fix double startup chime
                 - Change current averaging method for more precision
                 - Fix startup ramp speed adjustment
*2.05		 - Fix ramp tied to input frequency
*2.06    - fix input pullups
         - Remove half xfer insterrupt from servo routine
                                 - update running brake and brake on stop
*2.07    - Dead time change f4a
*2.08		 - Move zero crosss timing
*2.09    - filter out short zero crosses
*2.10    - Polling only below commutation intverval of 1500-2000us
				 - fix tune frequency again
*2.11    - RC-Car mode fix
*2.12    - Reduce Advance on hard braking
*2.13    - Remove Input capture filter for dshot2400
         - Change dshot 300 speed detection threshold
*2.14    - Reduce G071 zero cross checks
         - Assign all mcu's duty cycle resolution 2000 steps
*2.15    - Enforce 1/2 commutation interval as minimum for g071
         - Revert timing change on braking
				 - Add per target over-ride option to max duty cycle change.
				 - todo fix signal detection
*2.16    - add L431
				 - add variable auto timing
				 - add droneCAN
*/
#include "main.h"
#include "ADC.h"
#include "IO.h"
#include "common.h"
#include "comparator.h"
#include "dshot.h"
#include "eeprom.h"
#include "functions.h"
#include "peripherals.h"
#include "phaseouts.h"
#include "serial_telemetry.h"
#include "kiss_telemetry.h"
#include "hwci_perf.h"
#include "commutation.h"
#include "bemf_zc.h"
#include "control_loop.h"
#include "faults.h"
#include "settings.h"
#include "brushed.h"
#include "runtime_loop.h"
#include "motor_runtime.h"
#include "esc_state.h"
/* esc_state is also snapshotted by HWCI_PERF_MAIN_LOOP (needs esc_state.h). */

/* Control path modules + motor_runtime state + esc_state machine. */
#include "signal.h"
#include "sounds.h"
#include "targets.h"
#include <stdint.h>
#include <string.h>
#include <assert.h>

#ifndef NXP
#	ifdef USE_LED_STRIP
#		include "WS2812.h"
#	endif
#endif

#ifdef USE_CRSF_INPUT
#	include "crsf.h"
#endif

#if DRONECAN_SUPPORT
#	include "DroneCAN/DroneCAN.h"
#endif

#include "version.h"
#include "bootloader_update.h"

// firmware build options !! fixed speed and duty cycle modes are not to be used
// with sinusoidal startup !!

//#define FIXED_DUTY_MODE  // bypasses signal input and arming, uses a set duty
// cycle. For pumps, slot cars etc
//#define FIXED_DUTY_MODE_POWER 100     //
// 0-100 percent not used in fixed speed mode

// #define FIXED_SPEED_MODE  // bypasses input signal and runs at a fixed rpm
// using the speed control loop PID
//#define FIXED_SPEED_MODE_RPM  1000  //
// intended final rpm , ensure pole pair numbers are entered correctly in config
// tool.

// #define BRUSHED_MODE         // overrides all brushless config settings,
// enables two channels for brushed control
//#define GIMBAL_MODE     // also
// sinusoidal_startup needs to be on, maps input to sinusoidal angle.

/* Runtime globals: motor_runtime.c */

/* loadEEpromSettings / saveEEpromSettings / checkDeviceInfo -> settings.c */

int main(void)
{
#ifdef NXP
	initCorePeripherals();
	checkDeviceInfo();
	loadEEpromSettings();
	enableCorePeripherals();
	initAfterJump();
#else
	initAfterJump();
	checkDeviceInfo();
	initCorePeripherals();
	enableCorePeripherals();
	/* May program BL region and NVIC_SystemReset when image differs. */
	maybe_update_bootloader();
	loadEEpromSettings();
#endif

	if (VERSION_MAJOR != eepromBuffer.version.major || VERSION_MINOR != eepromBuffer.version.minor ||
	    EEPROM_VERSION > eepromBuffer.eeprom_version) {
		eepromBuffer.version.major = VERSION_MAJOR;
		eepromBuffer.version.minor = VERSION_MINOR;
		eepromBuffer.eeprom_version = EEPROM_VERSION;
		saveEEpromSettings();
	}

	if (eepromBuffer.dir_reversed == 1) {
		forward = 0;
	} else {
		forward = 1;
	}
	tim1_arr = TIMER1_MAX_ARR;
	if (!eepromBuffer.comp_pwm) {
		eepromBuffer.use_sine_start = 0; // sine start requires complementary pwm.
	}

	if (eepromBuffer.rc_car_reverse) { // overrides a whole lot of things!
		throttle_max_at_low_rpm = 1000;
		eepromBuffer.bi_direction = 1;
		eepromBuffer.use_sine_start = 0;
		low_rpm_throttle_limit = 1;
		eepromBuffer.variable_pwm = 0;
		// eepromBuffer.stall_protection = 1;
		eepromBuffer.comp_pwm = 0;
		eepromBuffer.stuck_rotor_protection = 0;
		minimum_duty_cycle = minimum_duty_cycle + 50;
		stall_protect_minimum_duty = stall_protect_minimum_duty + 50;
		min_startup_duty = min_startup_duty + 50;
	}

#ifdef MCU_F031
	GPIOF->BSRR = LL_GPIO_PIN_6; // uncomment to take bridge out of standby mode
	// and set oc level
	GPIOF->BRR = LL_GPIO_PIN_7; // out of standby mode
	GPIOA->BRR = LL_GPIO_PIN_11;
#endif
#ifdef MCU_G031
	GPIOA->BRR = LL_GPIO_PIN_11;
	GPIOA->BSRR = LL_GPIO_PIN_12; // Pa12 attached to enable on dev board
#endif
#ifdef USE_LED_STRIP
	send_LED_RGB(125, 0, 0);
#endif
#ifdef USE_RGB_LED
	setIndividualRGBLed(1, 0, 0);
#endif

#ifdef USE_CRSF_INPUT
	inputSet = 1;
	playStartupTune();
	MX_IWDG_Init();
	LL_IWDG_ReloadCounter(IWDG);
#else
#	if defined(FIXED_DUTY_MODE) || defined(FIXED_SPEED_MODE)
	MX_IWDG_Init();
	RELOAD_WATCHDOG_COUNTER();
	inputSet = 1;
	escToArmedIdle();
	adjusted_input = 48;
	newinput = 48;
	comStep(2);
#		ifdef FIXED_SPEED_MODE
	use_speed_control_loop = 1;
	eepromBuffer.use_sine_start = 0;
	target_e_com_time = 60000000 / FIXED_SPEED_MODE_RPM / (eepromBuffer.motor_poles / 2);
	input = 48;
#		endif

#	else
#		ifdef BRUSHED_MODE
	// bi_direction = 1;
	commutation_interval = 5000;
	eepromBuffer.use_sine_start = 0;
	maskPhaseInterrupts();
	playBrushedStartupTune();
#		else
#			ifdef MCU_AT415
	play_tone_flag = 5;
#			else
	playStartupTune();
#			endif
#		endif
	zero_input_count = 0;
	MX_IWDG_Init();
	RELOAD_WATCHDOG_COUNTER();
#		ifdef GIMBAL_MODE
	eepromBuffer.bi_direction = 1;
	eepromBuffer.use_sine_start = 1;
#		endif

#		ifdef USE_ADC_INPUT
	armed_count_threshold = 5000;
	inputSet = 1;

#		else
	// checkForHighSignal();     // will reboot if signal line is high for 10ms
	receiveDshotDma();
	if (drive_by_rpm) {
		use_speed_control_loop = 1;
	}
#		endif

#	endif // end fixed duty mode ifdef
#endif	       // end crsf input

#ifdef MCU_F051
	MCU_Id = DBGMCU->IDCODE &= 0xFFF;
	REV_Id = DBGMCU->IDCODE >> 16;

	if (REV_Id >= 4096) {
		temperature_offset = 0;
	} else {
		temperature_offset = 230;
	}

#endif
#ifdef NEUTRONRC_G071
	setInputPullDown();
#else
	setInputPullUp();
#endif

#ifdef USE_STARTUP_BOOST
	min_startup_duty = min_startup_duty + 200 + ((eepromBuffer.pwm_frequency * 100) / 24);
	minimum_duty_cycle = minimum_duty_cycle + 50 + ((eepromBuffer.pwm_frequency * 50) / 24);
	startup_max_duty_cycle = startup_max_duty_cycle + 400;
#endif

	uint16_t last_tim1_arr = 0; // force scale factor computation on first pass

	// minimum_duty_cycle is final at this point, precompute the input to duty
	// cycle slopes so setInput multiplies instead of calling map()
	throttle_duty_slope_q16 = (((uint32_t)(2000 - minimum_duty_cycle)) << 16) / (2047 - 47);
	sine_throttle_duty_slope_q16 = (((uint32_t)(2000 - (minimum_duty_cycle + 40))) << 16) / (2047 - 137);

	escReconcileFromFlags();

	while (1) {
		HWCI_PERF_MAIN_LOOP();
		e_com_time = ((commutation_intervals[0] + commutation_intervals[1] + commutation_intervals[2] + commutation_intervals[3] +
			       commutation_intervals[4] + commutation_intervals[5]) +
			      4) >>
			     1; // COMMUTATION INTERVAL IS 0.5US INCREMENTS

#if defined(FIXED_DUTY_MODE) || defined(FIXED_SPEED_MODE)
		setInput();
#endif

#ifdef NEED_INPUT_READY
#	ifdef MCU_F031
		if (input_ready) {
			setInput();
			input_ready = 0;
		}
#	else
		if (input_ready) {
			processDshot();
			input_ready = 0;
		}
#	endif
#endif
		if (zero_crosses < 5) {
			if (eepromBuffer.bi_direction) {
				min_bemf_counts_up = TARGET_MIN_BEMF_COUNTS + 1;
				min_bemf_counts_down = TARGET_MIN_BEMF_COUNTS + 1;
			} else {
				min_bemf_counts_up = TARGET_MIN_BEMF_COUNTS * 2;
				min_bemf_counts_down = TARGET_MIN_BEMF_COUNTS * 2;
			}
		} else {
			min_bemf_counts_up = TARGET_MIN_BEMF_COUNTS;
			min_bemf_counts_down = TARGET_MIN_BEMF_COUNTS;
		}

		RELOAD_WATCHDOG_COUNTER();

		runtimeUpdateVariablePwm(&last_tim1_arr);
		faultPollSignalTimeout();
#ifdef USE_CUSTOM_LED
		if ((input >= 47) && (input < 1947)) {
			if (ledcounter > (2000 >> forward)) {
				GPIOB->BSRR = LL_GPIO_PIN_3;
			} else {
				GPIOB->BRR = LL_GPIO_PIN_3;
			}
			if (ledcounter > (4000 >> forward)) {
				ledcounter = 0;
			}
		}
		if (input > 1947) {
			GPIOB->BSRR = LL_GPIO_PIN_3;
		}
		if (input < 47) {
			GPIOB->BRR = LL_GPIO_PIN_3;
		}
#endif

		if (tenkhzcounter > LOOP_FREQUENCY_HZ) { // 1s sample interval 10000
			consumed_current += (actual_current << 16) / 360;
			tenkhzcounter = 0;
		}

		faultUpdateBemfTimeoutPolicy();
		runtimeSampleBemfPreLevel();
		runtimeProcessDesyncCheck();
		runtimeUpdateDshotIrqPriority();
		runtimeSendTelemetryIfNeeded();
		runtimeProcessAdcAndProtections();
		runtimeMotorModeTick();
#ifdef BRUSHED_MODE
		runBrushedLoop();
#endif
#if DRONECAN_SUPPORT
		DroneCAN_update();
#endif
	}
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line
       number, tex: printf("Wrong parameters value: file %s on line %d\r\n", file,
       line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
