/*
 * settings.c - extracted from main.c (behavior-neutral)
 */

#include "settings.h"

#include "main.h"
#include "common.h"
#include "motor_runtime.h"
#include "eeprom.h"
#include "functions.h"
#include "peripherals.h"
#include "sounds.h"
#include "signal.h"
#include "targets.h"
#include "IO.h"
#include "version.h"
#include "pwm_app.h"

void loadEEpromSettings(void)
{
	read_flash_bin(eepromBuffer.buffer, eeprom_address, sizeof(eepromBuffer.buffer));
	if (eepromBuffer.eeprom_version < EEPROM_VERSION) {
		eepromBuffer.max_ramp = TARGET_DEFAULT_MAX_RAMP; // 0.1% per ms steps (see targets.h)
		eepromBuffer.minimum_duty_cycle = 1;		 // 0.2% to 51 percent
		eepromBuffer.disable_stick_calibration = 0;	 //
		eepromBuffer.absolute_voltage_cutoff = 10;	 // voltage level 1 to 100 in 0.5v increments
		eepromBuffer.current_P = 100;			 // 0-255
		eepromBuffer.current_I = 0;			 // 0-255
		eepromBuffer.current_D = 100;			 // 0-255
		eepromBuffer.active_brake_power = 0;		 // 1-5 percent duty cycle
		eepromBuffer.reserved_eeprom_3[0] = 0;		 //14-16  for crsf input
		eepromBuffer.reserved_eeprom_3[1] = 0;
		eepromBuffer.reserved_eeprom_3[2] = 0;
		eepromBuffer.reserved_eeprom_3[3] = 0;
	}
	// eepromBuffer.advance_level can either be set to 0-3 with config tools less than 1.90 or 10-42 with 1.90 or above
	if (eepromBuffer.advance_level > 42 || (eepromBuffer.advance_level < 10 && eepromBuffer.advance_level > 3)) {
		temp_advance = 16;
	}
	if (eepromBuffer.advance_level < 4) { // old format needs to be converted to 0-32 range
		temp_advance = (eepromBuffer.advance_level << 3);
		eepromBuffer.advance_level = temp_advance + 10;
	}
	if (eepromBuffer.advance_level < 43 && eepromBuffer.advance_level > 9) { // new format subtract 10 from advance
		temp_advance = eepromBuffer.advance_level - 10;
	}

	if (eepromBuffer.pwm_frequency < 145 && eepromBuffer.pwm_frequency > 7) {
		int divider = eepromBuffer.pwm_frequency * 100 / 6;
		TIMER1_MAX_ARR = TIM1_AUTORELOAD * 400 / divider;
		SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
	} else {
		tim1_arr = TIM1_AUTORELOAD;
		SET_AUTO_RELOAD_PWM(tim1_arr);
	}
	if (eepromBuffer.minimum_duty_cycle < 51 && eepromBuffer.minimum_duty_cycle > 0) {
		minimum_duty_cycle = eepromBuffer.minimum_duty_cycle * 10;
	} else {
		minimum_duty_cycle = 0;
	}
	if (eepromBuffer.startup_power < 151 && eepromBuffer.startup_power > 49) {
		min_startup_duty = minimum_duty_cycle + eepromBuffer.startup_power;
	} else {
		min_startup_duty = minimum_duty_cycle;
	}
	startup_max_duty_cycle = minimum_duty_cycle + 400;

	motor_kv = (eepromBuffer.motor_kv * 40) + 20;
#ifdef THREE_CELL_MAX
	motor_kv = motor_kv / 2;
#endif
#ifdef ONE_TWO_CELL_MAX
	motor_kv = motor_kv / 16;
#endif
	setVolume(2);
	if (eepromBuffer.eeprom_version > 0) { // these commands weren't introduced until eeprom version 1.
#ifdef CUSTOM_RAMP

#else
		if (eepromBuffer.beep_volume > 11) {
			setVolume(5);
		} else {
			setVolume(eepromBuffer.beep_volume);
		}
#endif
		servo_low_threshold = (eepromBuffer.servo.low_threshold * 2) + 750;    // anything below this point considered 0
		servo_high_threshold = (eepromBuffer.servo.high_threshold * 2) + 1750; // anything above this point considered 2000 (max)
		servo_neutral = (eepromBuffer.servo.neutral) + 1374;
		servo_dead_band = eepromBuffer.servo.dead_band;

		low_cell_volt_cutoff = eepromBuffer.low_cell_volt_cutoff + 250; // 2.5 to 3.5 volts per cell range

#ifndef HAS_HALL_SENSORS
		eepromBuffer.use_hall_sensors = 0;
#endif

		if (eepromBuffer.sine_mode_changeover_thottle_level < 5 ||
		    eepromBuffer.sine_mode_changeover_thottle_level > 25) { // sine mode changeover 5-25 percent throttle
			eepromBuffer.sine_mode_changeover_thottle_level = 5;
		}
		if (eepromBuffer.drag_brake_strength == 0 || eepromBuffer.drag_brake_strength > 10) { // drag brake 1-10
			eepromBuffer.drag_brake_strength = 10;
		}

		if (eepromBuffer.driving_brake_strength == 0 || eepromBuffer.driving_brake_strength > 9) { // motor brake 1-9
			eepromBuffer.driving_brake_strength = 10;
		}

		if (eepromBuffer.driving_brake_strength < 10) {
			dead_time_override = DEAD_TIME + (150 - (eepromBuffer.driving_brake_strength * 10));
			if (dead_time_override > 200) {
				dead_time_override = 200;
			}
			min_startup_duty = min_startup_duty + dead_time_override;
			minimum_duty_cycle = minimum_duty_cycle + dead_time_override;
			throttle_max_at_low_rpm = throttle_max_at_low_rpm + dead_time_override;
			startup_max_duty_cycle = startup_max_duty_cycle + dead_time_override;
			setPwmDeadTime(dead_time_override);
		}
		if (eepromBuffer.limits.temperature < 70 || eepromBuffer.limits.temperature > 140) {
			eepromBuffer.limits.temperature = 255;
		}

		if (eepromBuffer.limits.current > 0 && eepromBuffer.limits.current <= 100) {
			use_current_limit = 1;
		}

		currentPid.Kp = eepromBuffer.current_P * 2;
		currentPid.Ki = eepromBuffer.current_I;
		currentPid.Kd = eepromBuffer.current_D * 2;

		if (eepromBuffer.sine_mode_power == 0 || eepromBuffer.sine_mode_power > 10) {
			eepromBuffer.sine_mode_power = 5;
		}

		// unsinged int cant be less than 0
		if (eepromBuffer.input_type < 10) {
			switch (eepromBuffer.input_type) {
				case AUTO_IN:
					dshot = 0;
					servoPwm = 0;
					EDT_ARMED = 1;
					break;
				case DSHOT_IN:
					dshot = 1;
					EDT_ARMED = 1;
					break;
				case SERVO_IN:
					servoPwm = 1;
					break;
				case SERIAL_IN:
					break;
				case EDTARM_IN:
					EDT_ARM_ENABLE = 1;
					EDT_ARMED = 0;
					dshot = 1;
					break;
			};
		} else {
			dshot = 0;
			servoPwm = 0;
			EDT_ARMED = 1;
		}

		if (eepromBuffer.max_ramp < 10) {
			ramp_divider = 9;
			max_ramp_startup = eepromBuffer.max_ramp;
			max_ramp_low_rpm = eepromBuffer.max_ramp;
			max_ramp_high_rpm = eepromBuffer.max_ramp;
		} else {
			ramp_divider = 0;
			if ((eepromBuffer.max_ramp / 10) < max_ramp_startup) {
				max_ramp_startup = eepromBuffer.max_ramp / 10;
			}
			if ((eepromBuffer.max_ramp / 10) < max_ramp_low_rpm) {
				max_ramp_low_rpm = eepromBuffer.max_ramp / 10;
			}
			if ((eepromBuffer.max_ramp / 10) < max_ramp_high_rpm) {
				max_ramp_high_rpm = eepromBuffer.max_ramp / 10;
			}
		}

		if (motor_kv < 300) {
			low_rpm_throttle_limit = 0;
		}
		// guard divisions for an erased eeprom (motor_poles 0 or 0xff),
		// ARM hardware division returns 0 but it is UB in C
		uint8_t rpm_level_div = 0;
		if (eepromBuffer.motor_poles != 0) {
			rpm_level_div = 32 / eepromBuffer.motor_poles;
		}
		if (rpm_level_div != 0) {
			low_rpm_level = motor_kv / 100 / rpm_level_div;
			high_rpm_level = motor_kv / 12 / rpm_level_div;
		} else {
			low_rpm_level = 0;
			high_rpm_level = 0;
		}
	}
	reverse_speed_threshold = map(motor_kv, 300, 3000, 1000, 500);
	if (eepromBuffer.bi_direction) {
		polling_mode_changeover = POLLING_MODE_THRESHOLD / 2;
	} else {
		polling_mode_changeover = POLLING_MODE_THRESHOLD;
	}
}

void saveEEpromSettings(void)
{
	save_flash_nolib(eepromBuffer.buffer, sizeof(eepromBuffer.buffer), eeprom_address);
}

/*
  check device info from the bootloader, confirming pin code and eeprom location
 */
void __attribute__((noinline)) checkDeviceInfo(void)
{
#ifdef MCU_SITL
	// no bootloader device info page in SITL
	return;
#endif
#ifdef NXP
	uint32_t pflashBlockBase = 0U;
	uint32_t pflashTotalSize = 0U;
	uint32_t pflashSectorSize = 0U;

	//Get flash properties
	FLASH_API->flash_get_property(&s_flashDriver, kFLASH_PropertyPflashBlockBaseAddr, &pflashBlockBase);
	FLASH_API->flash_get_property(&s_flashDriver, kFLASH_PropertyPflashSectorSize, &pflashSectorSize);
	FLASH_API->flash_get_property(&s_flashDriver, kFLASH_PropertyPflashTotalSize, &pflashTotalSize);
#else
#	define DEVINFO_MAGIC1 0x5925e3da
#	define DEVINFO_MAGIC2 0x4eb863d9

	// Fixed bootloader address; GCC 12+ -Warray-bounds needs care.
	struct devinfo {
		uint32_t magic1;
		uint32_t magic2;
		uint8_t deviceInfo[9];
	};
	volatile const struct devinfo *devinfo = (volatile const struct devinfo *)(uintptr_t)(0x1000u - 32u);
#	if defined(__GNUC__) && (__GNUC__ >= 12)
#		pragma GCC diagnostic push
#		pragma GCC diagnostic ignored "-Warray-bounds"
#		pragma GCC diagnostic ignored "-Wstringop-overread"
#	endif
	const uint32_t magic1 = devinfo->magic1;
	const uint32_t magic2 = devinfo->magic2;
	const uint8_t eeprom_code = devinfo->deviceInfo[4];
#	if defined(__GNUC__) && (__GNUC__ >= 12)
#		pragma GCC diagnostic pop
#	endif
	if (magic1 != DEVINFO_MAGIC1 || magic2 != DEVINFO_MAGIC2) {
		// bootloader does not support this feature, nothing to do
		return;
	}
	// change eeprom_address based on the code in the bootloaders device info
	switch (eeprom_code) {
		case 0x1f:
			eeprom_address = 0x08007c00;
			break;
		case 0x35:
			eeprom_address = 0x0800f800;
			break;
		case 0x2b:
			eeprom_address = 0x0801f800;
			break;
	}
#endif

	// TODO: check pin code and reboot to bootloader if incorrect
}
