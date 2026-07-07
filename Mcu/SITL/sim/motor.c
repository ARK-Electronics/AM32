/*
  motor.c - trapezoidal BEMF BLDC motor, 3 phase bridge and battery model
  for AM32 SITL.

  The electrical model follows the approach of open-bldc-csim
  (https://github.com/open-bldc/open-bldc-csim, GPLv3+, Piotr
  Esden-Tempski): per phase currents with a solved star point voltage so
  the floating phase terminal voltage (and therefore the BEMF zero
  crossing seen by the comparator) is physical. Extended with fet Rds_on,
  body diode clamping of a floating phase that still carries current, and
  battery internal resistance.

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

static struct {
    double theta; // mechanical angle, rad
    double omega; // mechanical speed, rad/s
    double i[3]; // phase currents, A
    double ke; // V/(rad/s) mechanical
    double vbus; // battery terminal voltage after sag
    double ibus; // battery current, previous step (breaks the loop)
    // sensor averaging accumulators
    double acc_v, acc_i;
    uint32_t acc_n;
    unsigned rand_seed;
    // seqlock for the sensor snapshot
    volatile uint32_t seq;
    sitl_sensors_t sensors;
} m;

void motor_init(void)
{
    memset(&m, 0, sizeof(m));
    // per phase BEMF constant [V s/rad] from Kv [rpm/V]. Kv is defined
    // line to line and two phases conduct in series, so halve it
    m.ke = 0.5 * 60.0 / (TWO_PI * sitl_cfg.motor.kv);
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

void sitl_sensors_write(const sitl_sensors_t* in)
{
    m.seq++;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    m.sensors = *in;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    m.seq++;
}

void sitl_sensors_read(sitl_sensors_t* out)
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

    // gate states from the phase mode and the emulated PWM timer
    bool hi[3], lo[3];
    for (int p = 0; p < 3; p++) {
        const bool pwm = sitl_tim1_pwm_out(p, now_ns);
        switch (sitl_phase_mode[p]) {
        case SITL_PHASE_PWM:
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
    }

    // terminal voltages. A driven phase is tied to the rail through the
    // fet; an undriven phase carrying current is clamped by a body diode;
    // an undriven phase without current floats at e + v_star
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
            // body diode freewheeling
            v[p] = m.i[p] < 0 ? vbus + vf : -vf;
            conducting[p] = true;
        } else {
            m.i[p] = 0;
            v[p] = 0; // filled in after v_star is known
            conducting[p] = false;
        }
    }

    // star point voltage from the conducting phases (sum of currents is
    // zero and impedances are equal)
    double v_star = 0;
    int n_cond = 0;
    for (int p = 0; p < 3; p++) {
        if (conducting[p]) {
            v_star += v[p] - e[p];
            n_cond++;
        }
    }
    if (n_cond > 0) {
        v_star /= n_cond;
    }
    // a single conducting phase cannot drive current into the star point
    if (n_cond == 1) {
        for (int p = 0; p < 3; p++) {
            if (conducting[p] && !hi[p] && !lo[p]) {
                // lone diode phase: current decays through the diode
            }
        }
    }

    for (int p = 0; p < 3; p++) {
        if (!conducting[p]) {
            v[p] = e[p] + v_star;
        }
    }

    // phase current derivatives, semi-implicit euler
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

    // electromagnetic torque: tau = ke * sum(shape_p * i_p)
    double tau = 0;
    for (int p = 0; p < 3; p++) {
        tau += m.ke * shape[p] * m.i[p];
    }

    // load torques
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
            sitl_exti.PR |= line;
            const bool unmasked = (sitl_exti.IMR & line) != 0;
            if (unmasked) {
                sitl_irq_pend(SITL_IRQ_COMP);
            }
            motor_log_event(MEV_EDGE, out, unmasked, 0);
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
    fprintf(stderr,
        "SITL: desync %u at t=%.4fs avg=%u last_avg=%u zc=%u e_com_time=%d, recent events:\n",
        (unsigned)desync_happened, sitl_time_ns() * 1e-9,
        (unsigned)average_interval, (unsigned)last_average_interval,
        (unsigned)zero_crosses, e_com_time);
    dump_ring();
}

static void dump_ring(void)
{
    static const char* kinds[] = { "COMMUTATE", "EDGE", "BLANKED", "COMP_RUN", "ZC_ACCEPT", "MAINLOOP" };
    for (unsigned k = 0; k < COMM_RING; k++) {
        const unsigned idx = (comm_ring_pos + k) % COMM_RING;
        if (comm_ring[idx].t_ns == 0) {
            continue;
        }
        fprintf(stderr, "  t=%.6f %-9s thetae=%5.1f rpm=%6.0f ci=%u a=%u b=%u c=%u\n",
            comm_ring[idx].t_ns * 1e-9, kinds[comm_ring[idx].kind],
            (double)comm_ring[idx].thetae_deg, (double)comm_ring[idx].rpm,
            (unsigned)comm_ring[idx].ci,
            (unsigned)comm_ring[idx].a, (unsigned)comm_ring[idx].b,
            (unsigned)comm_ring[idx].c);
    }
}

void motor_get_state(double* theta, double* omega, double i[3])
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
        fprintf(stderr,
            "SITL settings: bidir=%u dir_rev=%u comp_pwm=%u poles=%u input_type=%u sine=%u brake_on_stop=%u\n",
            eepromBuffer.bi_direction, eepromBuffer.dir_reversed,
            eepromBuffer.comp_pwm, eepromBuffer.motor_poles,
            eepromBuffer.input_type, eepromBuffer.use_sine_start,
            eepromBuffer.brake_on_stop);
    }

    extern void sitl_can_stats(uint32_t stats[4]);
    uint32_t cs[4];
    sitl_can_stats(cs);

    fprintf(stderr,
        "SITL t=%.1fs x%.2f rpm=%.0f Vbus=%.2f Ibus=%.2f modes=%d%d%d in=%u newin=%u adj=%u armed=%d duty=%u step=%d zc=%u ci=%u run=%d old=%d bemf_to=%u cmd=%u\n",
        now_ns * 1e-9, (double)time_ratio, m.omega * 60.0 / TWO_PI,
        m.vbus, m.ibus,
        sitl_phase_mode[0], sitl_phase_mode[1], sitl_phase_mode[2],
        input, newinput, adjusted_input, armed, duty_cycle, step,
        (unsigned)zero_crosses, (unsigned)commutation_interval,
        running, old_routine, bemf_timeout_happened,
        (unsigned)cs[1]);
}
