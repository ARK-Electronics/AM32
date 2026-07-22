/*
  motor.c - trapezoidal BEMF BLDC motor, 3 phase bridge and battery model
  for AM32 SITL.

  The electrical model follows the approach of open-bldc-csim
  (https://github.com/open-bldc/open-bldc-csim, GPLv3+, Piotr
  Esden-Tempski), which in turn implements Kang & Yoo, "Switching
  Pattern-Independent Simulation Model for Brushless DC Motors", Journal
  of Power Electronics 11-2, 2011:
  https://jpels.org/digital-library/manuscript/file/17706/8_JPE-10238.pdf

  As in the paper, each step finds the conducting phases from the switch
  and diode states, computes the motor neutral as the average of (v - e)
  over the conducting phases (paper eq 10) and integrates
  v = R*i + (L-M)*di/dt + e + v_m for each of them (paper eq 1), so the
  floating phase terminal voltage (and therefore the BEMF zero crossing
  seen by the comparator) is physical for any switching pattern.

  Differences from the paper:
   - gate states come from the emulated TIM1 compare outputs and the AM32
     phase modes rather than switching functions, which adds dead time
     windows with body diode conduction
   - fets have Rds_on, and the diode Vf appears in the clamped terminal
     voltage, not only in the diode on/off conditions
   - the battery has internal resistance, so Vdc sags with load instead
     of being stiff
   - the load model adds a k*omega^2 propeller torque and static friction
     to the paper's viscous damping
   - torque uses the flux linkage form kt*sum(shape*i) (the second form
     of paper eq 2), which is valid at omega = 0
   - integration is forward euler at 500ns steps rather than ode45 at
     2.5us

  Conventions:
   - phase currents are positive INTO the motor terminal
   - terminal voltages are referenced to battery negative
   - theta is the mechanical rotor angle in radians
 */

#include "motor.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sitl.h"
#include "sitl_config.h"
#include "eeprom.h"

#define TWO_PI 6.283185307179586

// which EXTI line the active comparator drives, owned by comparator.c
extern uint32_t current_EXTI_LINE;

static void check_desync_dump(void);
static void dump_ring(void);

/*
  zero-cross fault injection (state port cmd 3): suppress comparator EXTI
  delivery so the firmware's missed-ZC blind-step path can be exercised
  deterministically. The comparator OUTPUT keeps tracking the physics (poll
  mode and the confirm loop read the level directly and must stay honest) -
  only the edge -> EXTI pend -> NVIC delivery is dropped, which is exactly
  what a crossing invisible to the interrupt path looks like.
  mode 0: off, 1: drop every delivery, 2: drop deliveries during every
  other commutation window (alternating real/missed - the demag signature
  the miss-rate bucket exists for).
 */
static uint8_t zc_fault_mode;
static uint64_t zc_fault_end_ns;
static uint32_t zc_dropped_edges;
static uint32_t commutation_count; // total comStep calls, real + blind

void motor_zc_fault(uint8_t mode, uint32_t duration_us)
{
	zc_fault_mode = mode;
	zc_fault_end_ns = sitl_time_ns() + (uint64_t)duration_us * 1000ULL;
	fprintf(stderr, "SITL: zc fault mode %u for %u us\n", mode, duration_us);
}

uint32_t motor_zc_dropped(void)
{
	return zc_dropped_edges;
}

static struct {
	double theta; // mechanical angle, rad
	double omega; // mechanical speed, rad/s
	double i[3];  // phase currents, A
	double ke;    // V/(rad/s) mechanical
	double vbus;  // battery terminal voltage after sag
	double ibus;  // battery current, previous step (breaks the loop)
	// dead time tracking: per phase last PWM level and the end of the
	// current both-off window
	bool pwm_last[3];
	uint64_t dead_until[3];
	// last terminal voltages for the state stream
	double v_term[3];
	// sensor averaging accumulators
	double acc_v, acc_i;
	uint32_t acc_n;
	unsigned rand_seed;
	// seqlock for the sensor snapshot
	volatile uint32_t seq;
	sitl_sensors_t sensors;
} m;

void motor_config_changed(void)
{
	// per phase BEMF constant [V s/rad] from Kv [rpm/V]. Kv is defined
	// line to line and two phases conduct in series, so halve it. All
	// other motor parameters are read from sitl_cfg on every step
	m.ke = 0.5 * 60.0 / (TWO_PI * sitl_cfg.motor.kv);
}

void motor_add_signals(double acc[8])
{
	for (int p = 0; p < 3; p++) {
		acc[p] += m.i[p];
		acc[3 + p] += m.v_term[p];
	}
	acc[6] += m.vbus;
	acc[7] += m.ibus;
}

void motor_get_live_state(float *omega, float *theta, float *theta_e, float i[3], float v[3], float *vbus, float *ibus)
{
	for (int p = 0; p < 3; p++) {
		v[p] = (float)m.v_term[p];
	}
	const int pole_pairs = sitl_cfg.motor.poles / 2;
	*omega = (float)m.omega;
	double th = fmod(m.theta, TWO_PI);
	if (th < 0) {
		th += TWO_PI;
	}
	*theta = (float)th;
	double the = fmod(m.theta * pole_pairs, TWO_PI);
	if (the < 0) {
		the += TWO_PI;
	}
	*theta_e = (float)the;
	for (int p = 0; p < 3; p++) {
		i[p] = (float)m.i[p];
	}
	*vbus = (float)m.vbus;
	*ibus = (float)m.ibus;
}

void motor_init(void)
{
	memset(&m, 0, sizeof(m));
	motor_config_changed();
	m.vbus = sitl_cfg.battery.voltage;
	m.rand_seed = 12345;
	m.sensors.bus_voltage = m.vbus;
	m.sensors.temperature_c = sitl_cfg.esc.temperature_c;
}

/*
  normalised trapezoidal BEMF shape over one electrical revolution.
  Rising zero crossing at 0, falling at pi, flat top from pi/6..5pi/6
 */
static double trap_shape(double thetae)
{
	thetae = fmod(thetae, TWO_PI);
	if (thetae < 0) {
		thetae += TWO_PI;
	}
	const double s = M_PI / 6; // 30 degree ramp
	if (thetae < s) {
		return thetae / s;
	}
	if (thetae < M_PI - s) {
		return 1.0;
	}
	if (thetae < M_PI + s) {
		return (M_PI - thetae) / s;
	}
	if (thetae < TWO_PI - s) {
		return -1.0;
	}
	return (thetae - TWO_PI) / s;
}

void sitl_sensors_write(const sitl_sensors_t *in)
{
	m.seq++;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	m.sensors = *in;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	m.seq++;
}

void sitl_sensors_read(sitl_sensors_t *out)
{
	for (;;) {
		const uint32_t s1 = m.seq;
		__atomic_thread_fence(__ATOMIC_SEQ_CST);
		*out = m.sensors;
		__atomic_thread_fence(__ATOMIC_SEQ_CST);
		if (s1 == m.seq && (s1 & 1) == 0) {
			return;
		}
	}
}

void motor_step(uint64_t now_ns, uint32_t dt_ns)
{
	const double dt = dt_ns * 1e-9;
	const int pole_pairs = sitl_cfg.motor.poles / 2;
	const double R = sitl_cfg.motor.resistance;
	const double L_eff = sitl_cfg.motor.inductance - sitl_cfg.motor.mutual_inductance;
	const double rds = sitl_cfg.esc.rds_on;
	const double vf = sitl_cfg.esc.diode_vf;
	const double i_eps = 0.01; // A, diode turn off threshold

	// battery sag from last step's bus current
	double vbus = sitl_cfg.battery.voltage - m.ibus * sitl_cfg.battery.resistance;
	if (vbus < 0) {
		vbus = 0;
	}
	m.vbus = vbus;

	// BEMF per phase
	// phase order such that the AM32 comStep sequence 1..6 advances the
	// field by +60 degrees electrical per step (verified by detent test)
	const double thetae = m.theta * pole_pairs;
	double e[3], shape[3];
	for (int p = 0; p < 3; p++) {
		shape[p] = trap_shape(thetae + p * (TWO_PI / 3.0));
		e[p] = m.ke * m.omega * shape[p];
	}

	// gate states from the phase mode and the emulated PWM timer. On a
	// complementary switched phase both fets are off for the configured
	// dead time after each PWM edge and the body diode conducts, as on
	// hardware
	const uint32_t dead_ns = sitl_tim1_dead_time_ns();
	bool hi[3], lo[3];
	for (int p = 0; p < 3; p++) {
		const bool pwm = sitl_tim1_pwm_out(p, now_ns);
		switch (sitl_phase_mode[p]) {
			case SITL_PHASE_PWM:
				if (pwm != m.pwm_last[p]) {
					m.pwm_last[p] = pwm;
					m.dead_until[p] = now_ns + dead_ns;
				}
				if (now_ns < m.dead_until[p]) {
					hi[p] = false;
					lo[p] = false;
					break;
				}
				hi[p] = pwm;
				lo[p] = !pwm;
				break;
			case SITL_PHASE_PWM_NOCOMP:
				hi[p] = pwm;
				lo[p] = false;
				break;
			case SITL_PHASE_LOW:
				hi[p] = false;
				lo[p] = true;
				break;
			case SITL_PHASE_BRAKE_PWM:
				hi[p] = false;
				lo[p] = !pwm;
				break;
			case SITL_PHASE_FLOAT:
			default:
				hi[p] = false;
				lo[p] = false;
				break;
		}
		if (sitl_phase_mode[p] != SITL_PHASE_PWM) {
			m.pwm_last[p] = pwm;
			m.dead_until[p] = 0;
		}
	}

	// terminal voltages (paper eq 9); conducting[] is the excited set of
	// paper eq 8. A driven phase is tied to the rail through the fet; an
	// undriven phase carrying current is clamped by a body diode; an
	// undriven phase without current floats at e + v_star
	double v[3];
	double r_eff[3];
	bool conducting[3];
	for (int p = 0; p < 3; p++) {
		r_eff[p] = R;
		if (hi[p]) {
			v[p] = vbus;
			r_eff[p] += rds;
			conducting[p] = true;
		} else if (lo[p]) {
			v[p] = 0;
			r_eff[p] += rds;
			conducting[p] = true;
		} else if (fabs(m.i[p]) > i_eps) {
			// body diode freewheeling (paper eq 7, current condition)
			v[p] = m.i[p] < 0 ? vbus + vf : -vf;
			conducting[p] = true;
		} else {
			m.i[p] = 0;
			v[p] = 0; // filled in after v_star is known
			conducting[p] = false;
		}
	}

	// star point voltage from the conducting phases (paper eq 10; sum of
	// currents is zero and impedances are equal). With nothing conducting
	// the network floats; centre it on the bus as a symmetric bridge does
	double v_star = 0;
	int n_cond = 0;
	for (int p = 0; p < 3; p++) {
		if (conducting[p]) {
			v_star += v[p] - e[p];
			n_cond++;
		}
	}
	v_star = n_cond > 0 ? v_star / n_cond : 0.5 * vbus;

	// paper eq 7 second condition / fig 2(e): a body diode also turns on
	// when the floating terminal voltage e + v_m would exceed the rails,
	// even with no phase current. This clamps the floating phase at high
	// BEMF and lets a windmilling motor rectify into the battery. Each
	// new clamp moves the star point, so iterate
	for (int pass = 0; pass < 3; pass++) {
		bool changed = false;
		for (int p = 0; p < 3; p++) {
			if (conducting[p]) {
				continue;
			}
			const double vt = e[p] + v_star;
			if (vt > vbus + vf) {
				v[p] = vbus + vf;
			} else if (vt < -vf) {
				v[p] = -vf;
			} else {
				continue;
			}
			conducting[p] = true;
			changed = true;
		}
		if (!changed) {
			break;
		}
		v_star = 0;
		n_cond = 0;
		for (int p = 0; p < 3; p++) {
			if (conducting[p]) {
				v_star += v[p] - e[p];
				n_cond++;
			}
		}
		v_star /= n_cond;
	}

	// an open phase floats at e + v_m (paper eq 9)
	for (int p = 0; p < 3; p++) {
		if (!conducting[p]) {
			v[p] = e[p] + v_star;
		}
	}

	for (int p = 0; p < 3; p++) {
		m.v_term[p] = v[p];
	}

	// phase current derivatives (paper eq 1), forward euler
	for (int p = 0; p < 3; p++) {
		if (!conducting[p]) {
			continue;
		}
		const double di = (v[p] - r_eff[p] * m.i[p] - e[p] - v_star) / L_eff;
		m.i[p] += di * dt;
	}
	// with exactly two conducting phases KCL forces them equal and
	// opposite; project out any numerical drift
	if (n_cond == 2) {
		int a = -1, b = -1;
		for (int p = 0; p < 3; p++) {
			if (conducting[p]) {
				if (a < 0) {
					a = p;
				} else {
					b = p;
				}
			}
		}
		const double ic = 0.5 * (m.i[a] - m.i[b]);
		m.i[a] = ic;
		m.i[b] = -ic;
	} else if (n_cond < 2) {
		for (int p = 0; p < 3; p++) {
			m.i[p] = 0;
		}
	}

	// electromagnetic torque: tau = ke * sum(shape_p * i_p) (paper eq 2)
	double tau = 0;
	for (int p = 0; p < 3; p++) {
		tau += m.ke * shape[p] * m.i[p];
	}

	// load torques and motion (paper eq 3, plus k*omega^2 propeller
	// load and static friction)
	const double w = m.omega;
	double tau_load = sitl_cfg.motor.damping * w + sitl_cfg.motor.load_k_omega2 * w * fabs(w);
	double tau_net = tau - tau_load;
	const double sf = sitl_cfg.motor.static_friction;
	if (fabs(w) < 0.5) {
		// static friction dead band
		if (fabs(tau_net) <= sf) {
			tau_net = 0;
			m.omega = 0;
		} else {
			tau_net -= (tau_net > 0 ? sf : -sf);
		}
	} else {
		tau_net -= (w > 0 ? sf : -sf);
	}
	m.omega += tau_net / sitl_cfg.motor.inertia * dt;
	m.theta += m.omega * dt;
	if (m.theta > TWO_PI || m.theta < -TWO_PI) {
		m.theta = fmod(m.theta, TWO_PI);
	}

	// battery current: everything sourced from the positive rail
	double ibus = 0;
	for (int p = 0; p < 3; p++) {
		if (conducting[p] && v[p] >= vbus - 1e-9) {
			ibus += m.i[p];
		}
	}
	m.ibus = ibus;

	// comparator: virtual neutral against the floating phase terminal
	const double v_neutral = (v[0] + v[1] + v[2]) / 3.0;
	const double v_float = v[sitl_comp_phase];
	double diff_mv = (v_neutral - v_float) * 1000.0;
	if (sitl_cfg.sim.comparator_noise_mv > 0) {
		const double r = (double)rand_r(&m.rand_seed) / RAND_MAX - 0.5;
		diff_mv += r * 2.0 * sitl_cfg.sim.comparator_noise_mv;
	}
	const double hyst = sitl_cfg.sim.comparator_hysteresis_mv * 0.5;
	uint8_t out = sitl_comp_out;
	if (out) {
		out = diff_mv > -hyst;
	} else {
		out = diff_mv > hyst;
	}
	if (out != sitl_comp_out) {
		sitl_comp_out = out;
		const uint32_t line = current_EXTI_LINE;
		const bool rising_edge = out != 0;
		if ((rising_edge && (sitl_exti.RTSR & line)) || (!rising_edge && (sitl_exti.FTSR & line))) {
			if (zc_fault_mode && now_ns < zc_fault_end_ns && (zc_fault_mode == 1 || (commutation_count & 1u) == 0u)) {
				// injected missed crossing: edge never reaches EXTI.
				// Alternating mode keys off the commutation counter
				// (blind steps increment it too), so a blind step in a
				// dropped window flips the parity and the next window
				// delivers - real/missed/real/missed indefinitely.
				zc_dropped_edges++;
				motor_log_event(MEV_EDGE, out, 0, 1);
			} else {
				sitl_exti.PR |= line;
				const bool unmasked = (sitl_exti.IMR & line) != 0;
				if (unmasked) {
					sitl_irq_pend(SITL_IRQ_COMP);
				}
				motor_log_event(MEV_EDGE, out, unmasked, 0);
			}
		}
	}

	check_desync_dump();

	// sensor averaging, snapshot every 64 steps
	m.acc_v += m.vbus;
	m.acc_i += ibus > 0 ? ibus : 0;
	m.acc_n++;
	if (m.acc_n >= 64) {
		sitl_sensors_t s;
		s.bus_voltage = (float)(m.acc_v / m.acc_n);
		s.bus_current = (float)(m.acc_i / m.acc_n);
		s.temperature_c = sitl_cfg.esc.temperature_c;
		s.rpm = (float)(m.omega * 60.0 / TWO_PI);
		sitl_sensors_write(&s);
		m.acc_v = m.acc_i = 0;
		m.acc_n = 0;
	}
}

/*
  commutation debug ring: comStep calls in here; on a firmware desync the
  recent history is dumped so the failure can be analysed
 */
#define COMM_RING 96
static struct {
	uint64_t t_ns;
	float thetae_deg; // electrical angle
	float rpm;
	uint32_t ci;
	uint32_t a, b, c;
	uint8_t kind;
} comm_ring[COMM_RING];
static unsigned comm_ring_pos;

void motor_log_event(int kind, uint32_t a, uint32_t b, uint32_t c)
{
	extern volatile uint32_t commutation_interval;
	const int pole_pairs = sitl_cfg.motor.poles / 2;
	double deg = fmod(m.theta * pole_pairs * 180.0 / M_PI, 360.0);
	if (deg < 0) {
		deg += 360;
	}
	const unsigned idx = __atomic_fetch_add(&comm_ring_pos, 1, __ATOMIC_SEQ_CST) % COMM_RING;
	comm_ring[idx].t_ns = sitl_time_ns();
	comm_ring[idx].thetae_deg = (float)deg;
	comm_ring[idx].rpm = (float)(m.omega * 60.0 / TWO_PI);
	comm_ring[idx].ci = commutation_interval;
	comm_ring[idx].a = a;
	comm_ring[idx].b = b;
	comm_ring[idx].c = c;
	comm_ring[idx].kind = (uint8_t)kind;
}

void motor_log_mainloop(void)
{
	extern volatile uint32_t average_interval;
	extern uint32_t last_average_interval;
	static uint64_t last_ns;
	const uint64_t now = sitl_time_ns();
	const uint32_t gap_us = (uint32_t)((now - last_ns) / 1000ULL);
	last_ns = now;
	// only log iterations after a gap to avoid flooding the ring
	if (gap_us >= 50) {
		motor_log_event(MEV_MAINLOOP, average_interval, last_average_interval, gap_us);
	}
}

void motor_log_commutation(int step)
{
	extern volatile uint16_t duty_cycle;
	extern uint16_t duty_cycle_maximum;
	commutation_count++;
	motor_log_event(MEV_COMMUTATE, (uint32_t)step, duty_cycle, duty_cycle_maximum);
}

static void check_desync_dump(void)
{
	extern uint32_t desync_happened;
	static uint32_t last_desync;
	static int dumps;
	if (desync_happened == last_desync) {
		return;
	}
	last_desync = desync_happened;
	if (dumps >= 3) {
		return;
	}
	dumps++;
	extern volatile uint32_t average_interval;
	extern uint32_t last_average_interval;
	extern volatile uint32_t zero_crosses;
	extern int e_com_time;
	fprintf(stderr, "SITL: desync %u at t=%.4fs avg=%u last_avg=%u zc=%u e_com_time=%d, recent events:\n", (unsigned)desync_happened,
		sitl_time_ns() * 1e-9, (unsigned)average_interval, (unsigned)last_average_interval, (unsigned)zero_crosses, e_com_time);
	dump_ring();
}

static void dump_ring(void)
{
	static const char *kinds[] = {"COMMUTATE", "EDGE", "BLANKED", "COMP_RUN", "ZC_ACCEPT", "MAINLOOP"};
	for (unsigned k = 0; k < COMM_RING; k++) {
		const unsigned idx = (comm_ring_pos + k) % COMM_RING;
		if (comm_ring[idx].t_ns == 0) {
			continue;
		}
		fprintf(stderr, "  t=%.6f %-9s thetae=%5.1f rpm=%6.0f ci=%u a=%u b=%u c=%u\n", comm_ring[idx].t_ns * 1e-9,
			kinds[comm_ring[idx].kind], (double)comm_ring[idx].thetae_deg, (double)comm_ring[idx].rpm,
			(unsigned)comm_ring[idx].ci, (unsigned)comm_ring[idx].a, (unsigned)comm_ring[idx].b, (unsigned)comm_ring[idx].c);
	}
}

void motor_get_state(double *theta, double *omega, double i[3])
{
	*theta = m.theta;
	*omega = m.omega;
	for (int p = 0; p < 3; p++) {
		i[p] = m.i[p];
	}
}

void motor_print_state(uint64_t now_ns, float time_ratio)
{
	// firmware state, for debug output only
	extern uint16_t input;
	extern volatile char armed;
	extern char step;
	extern volatile uint32_t zero_crosses;
	extern volatile uint32_t commutation_interval;
	extern volatile uint16_t duty_cycle;
	extern uint8_t bemf_timeout_happened;
	extern uint8_t running;
	extern char old_routine;
	extern volatile uint16_t newinput;
	extern uint16_t adjusted_input;
	extern EEprom_t eepromBuffer;
	static bool printed_settings;
	if (!printed_settings) {
		printed_settings = true;
		fprintf(stderr, "SITL settings: bidir=%u dir_rev=%u comp_pwm=%u poles=%u input_type=%u sine=%u brake_on_stop=%u\n",
			eepromBuffer.bi_direction, eepromBuffer.dir_reversed, eepromBuffer.comp_pwm, eepromBuffer.motor_poles,
			eepromBuffer.input_type, eepromBuffer.use_sine_start, eepromBuffer.brake_on_stop);
	}

	extern void sitl_can_stats(uint32_t stats[4]);
	uint32_t cs[4];
	sitl_can_stats(cs);

	fprintf(stderr,
		"SITL t=%.1fs x%.2f rpm=%.0f Vbus=%.2f Ibus=%.2f modes=%d%d%d in=%u newin=%u adj=%u armed=%d duty=%u step=%d zc=%u ci=%u "
		"run=%d old=%d bemf_to=%u cmd=%u\n",
		now_ns * 1e-9, (double)time_ratio, m.omega * 60.0 / TWO_PI, m.vbus, m.ibus, sitl_phase_mode[0], sitl_phase_mode[1],
		sitl_phase_mode[2], input, newinput, adjusted_input, armed, duty_cycle, step, (unsigned)zero_crosses,
		(unsigned)commutation_interval, running, old_routine, bemf_timeout_happened, (unsigned)cs[1]);
}
