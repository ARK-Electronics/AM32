/*
 * motor_runtime.h - shared motor-control runtime state
 *
 * Definitions live in motor_runtime.c. Modules that need these symbols
 * include this header (or common.h / signal.h for the older shared set).
 */
#ifndef MOTOR_RUNTIME_H_
#define MOTOR_RUNTIME_H_

#include <stdint.h>
#include "common.h"

/* --- commutation / BEMF --- */
extern char step;
extern volatile char rising;
/* forward is in common.h */
extern char desync_check;
extern volatile uint8_t bemfcounter;
extern volatile uint8_t zcfound;
extern volatile uint32_t commutation_interval;
extern volatile uint16_t commutation_intervals[6];
extern volatile uint32_t average_interval;
extern uint32_t last_average_interval;
extern volatile uint16_t lastzctime;
extern volatile uint16_t thiszctime;
extern volatile uint16_t waitTime;
extern uint16_t advance;
extern uint8_t temp_advance;
extern uint8_t auto_advance_level;
extern volatile char old_routine;
extern volatile uint32_t zero_crosses;
extern volatile uint32_t polling_mode_changeover;
/* Missed-ZC blind-step fallback (bemf_zc.c) */
extern volatile uint8_t zc_deadline_armed;
extern volatile uint8_t zc_blind_steps;
extern volatile uint8_t zc_miss_bucket;
extern volatile uint8_t zc_pre_seen;
extern volatile uint8_t zc_demag_run;
extern volatile uint32_t zc_demag_accepts;
/* Blind-grind rail (faults.c): blind steps this 100 ms window / cut hold */
extern volatile uint8_t zc_blind_window_count;
extern volatile uint16_t zc_grind_hold_ms;
extern uint8_t filter_level;
extern uint8_t bad_count;
extern uint8_t bad_count_threshold;
extern uint8_t min_bemf_counts_up;
extern uint8_t min_bemf_counts_down;
extern char prop_brake_active;
extern uint8_t changeover_step;
extern uint8_t stuckcounter;
/* uint32_t always: DroneCAN telemetry and SITL stats both use 32-bit. */
extern uint32_t desync_happened;
/* Cross-episode desync rail (faults.c); saturating charge, time-based drain. */
extern volatile uint8_t desync_episode_bucket;
extern volatile uint16_t desync_restart_holdoff_ms;

/* --- duty / throttle --- */
extern volatile uint16_t duty_cycle;
extern uint16_t duty_cycle_setpoint;
extern uint16_t last_duty_cycle;
extern volatile uint16_t duty_cycle_maximum;
extern uint16_t minimum_duty_cycle;
extern uint16_t min_startup_duty;
extern uint16_t startup_max_duty_cycle;
extern uint16_t stall_protect_minimum_duty;
extern uint16_t adjusted_duty_cycle;
extern volatile uint16_t adjusted_input;
extern volatile uint16_t input;
extern volatile uint16_t newinput;
extern volatile char armed;
extern volatile uint8_t running;
extern volatile char stepper_sine;
extern char maximum_throttle_change_ramp;
extern uint8_t max_duty_cycle_change;
extern volatile uint8_t max_ramp_startup;
extern volatile uint8_t max_ramp_low_rpm;
extern volatile uint8_t max_ramp_high_rpm;
extern volatile uint8_t ramp_divider;
extern uint16_t ramp_count;
extern volatile uint32_t pwm_to_arr_scale_q16;
extern volatile uint32_t throttle_duty_slope_q16;
extern volatile uint32_t sine_throttle_duty_slope_q16;
extern volatile uint16_t tim1_arr;
extern uint16_t prop_brake_duty_cycle;

/* --- control / PID --- */
extern fastPID currentPid;
extern fastPID speedPid;
extern fastPID stallPid;
extern char use_speed_control_loop;
extern char use_current_limit;
extern int16_t use_current_limit_adjust;
extern int32_t input_override;
extern int32_t stall_protection_adjust;
extern uint16_t stall_protect_target_interval;
extern uint16_t target_e_com_time;
extern uint8_t drive_by_rpm;
extern uint32_t MAXIMUM_RPM_SPEED_CONTROL;
extern uint32_t MINIMUM_RPM_SPEED_CONTROL;
extern char brushed_direction_set;
extern char reversing_dead_band;
extern uint16_t reverse_speed_threshold;
extern uint16_t enter_sine_angle;
extern char return_to_center;
extern char do_once_sinemode;
extern uint16_t current_angle;
extern uint16_t desired_angle;
extern int16_t phase_A_position;
extern int16_t phase_B_position;
extern int16_t phase_C_position;
extern const int16_t pwmSin[];
extern uint16_t step_delay;
extern uint16_t gate_drive_offset;
extern uint16_t motor_kv;
extern uint8_t dead_time_override;
extern uint32_t MCU_Id;
extern uint32_t REV_Id;

/* --- sensing / telemetry tick --- */
extern uint16_t ADC_raw_current;
extern uint16_t ADC_raw_volts;
extern uint16_t ADC_raw_input;
#ifdef NXP
extern uint16_t ADC_raw_temp[];
#else
extern uint16_t ADC_raw_temp;
#endif
extern uint16_t ADC_raw_ntc;
extern uint16_t ADC_smoothed_input;
extern int16_t converted_degrees;
extern uint8_t temperature_offset;
extern uint16_t smoothedcurrent;
extern uint8_t readIndex;
extern uint32_t total;
extern uint16_t readings[];
extern const uint8_t numReadings;
extern char cell_count;
extern uint16_t e_rpm;
extern uint16_t k_erpm;
extern volatile int e_com_time;
extern char bemf_timeout;
extern uint8_t bemf_timeout_happened;
extern uint16_t armed_timeout_count;
extern uint16_t one_khz_loop_counter;
extern uint16_t ledcounter;
extern volatile uint16_t tenkhzcounter;
extern uint16_t telem_ms_count;
extern uint8_t telemetry_interval_ms;
extern volatile uint16_t signaltimeout;
extern char dshot;
extern uint8_t dshotcommand;
extern uint8_t last_dshot_command;
extern char play_tone_flag;
extern char low_rpm_throttle_limit;
extern uint16_t low_rpm_level;
extern uint16_t high_rpm_level;
extern uint16_t throttle_max_at_low_rpm;
extern uint16_t throttle_max_at_high_rpm;
extern char fast_accel;
extern char fast_deccel;
extern uint16_t servo_low_threshold;
extern uint16_t servo_high_threshold;
extern uint16_t servo_neutral;
extern uint8_t servo_dead_band;
extern volatile uint8_t PROCESS_ADC_FLAG;
extern int32_t consumed_current;
extern int32_t smoothed_raw_current;
extern char send_esc_info_flag;
extern uint16_t VOLTAGE_DIVIDER;
extern char LOW_VOLTAGE_CUTOFF;
extern uint16_t low_cell_volt_cutoff;
extern uint16_t low_voltage_count;
/* degrees_celsius, battery_voltage, actual_current: common.h / signal.h */

#endif /* MOTOR_RUNTIME_H_ */
