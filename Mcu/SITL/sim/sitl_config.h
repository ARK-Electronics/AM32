/*
  sitl_config.h - JSON file + command line configuration for AM32 SITL
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct {
	struct {
		float kv;		 // rpm per volt
		int poles;		 // magnetic poles (not pole pairs)
		float resistance;	 // phase resistance, ohm
		float inductance;	 // phase self inductance, henry
		float mutual_inductance; // henry (negative or zero)
		float inertia;		 // rotor+prop inertia, kg m^2
		float damping;		 // Nm/(rad/s)
		float static_friction;	 // Nm
		float load_k_omega2;	 // propeller load: Nm/(rad/s)^2
	} motor;
	struct {
		float voltage;	  // open circuit volts
		float resistance; // internal resistance, ohm
	} battery;
	struct {
		float rds_on;	     // fet on resistance, ohm
		float diode_vf;	     // body diode forward voltage
		float temperature_c; // reported temperature
	} esc;
	struct {
		uint32_t physics_dt_ns; // integration step
		uint32_t loop_time_ns;	// firmware main loop pacing sleep
		uint32_t isr_read_ns;	// cost of a register read in interrupt context
		float comparator_noise_mv;
		float comparator_hysteresis_mv;
		bool watchdog_enabled;
	} sim;

	// runtime options
	float speedup;	// 0 = free run
	int input_port; // UDP port for PWM/DShot input, 0 disables
	int state_port; // UDP port for state streaming/model control, 0 disables
	bool bind_any;	// bind input/state ports on all interfaces, not loopback
	const char *eeprom_path;
	const char *can_uri;
	const char *uid; // optional fixed unique ID string
	int node_id;	 // -1 = leave to eeprom/DNA
	int input_type;	 // eeprom INPUT_SIGNAL_TYPE override, -1 = leave
	bool verbose;
	bool nosleep;  // busy wait instead of sleeping, for timing accuracy
	bool realtime; // SCHED_FIFO for both threads
} sitl_config_t;

extern sitl_config_t sitl_cfg;

// parse CLI and optional JSON config, exits on error
void sitl_config_init(int argc, char **argv);

// runtime reload of the motor/battery/esc sections from a JSON file
// (sim section ignored). Returns false on error, never exits
bool sitl_config_reload(const char *path);
