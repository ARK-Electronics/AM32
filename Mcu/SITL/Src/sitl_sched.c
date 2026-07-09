/*
  sitl_sched.c - simulated time, interrupt delivery and pacing for AM32 SITL

  The sim thread advances simulated time in fixed physics steps. Emulated
  interrupts are delivered by suspending the firmware thread with SIGUSR1
  (parking it on a semaphore) and running the handler in the sim thread,
  which reproduces the run-to-completion, mainline-frozen semantics of real
  interrupts. __disable_irq()/__enable_irq() map onto an atomic PRIMASK
  flag; while set, events stay pending exactly as on hardware.
 */

#include "sitl.h"
#include "sitl_config.h"

#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef __linux__
#include <sys/prctl.h>
#endif
#include <time.h>
#include <unistd.h>

#include "motor.h"

static volatile uint64_t sim_time_ns_v;
static pthread_t sim_thread_id;
static pthread_t fw_thread_id;

static volatile int primask; // 1 = interrupts disabled, atomic stores only

// simulated time granted by firmware-thread timer reads while it holds
// PRIMASK (a startup tune busy-waits on the utility timer under
// __disable_irq, so time must still advance for it). Reset when the
// critical section ends
static volatile uint64_t fw_grant_ns;
static volatile uint32_t irq_pending;
static volatile uint32_t irq_enabled;
static volatile uint8_t irq_prio[SITL_IRQ_MAX];


static sem_t park_sem, resume_sem;

static volatile uint64_t watchdog_last_reload_ns;
static volatile bool watchdog_running;
#define WATCHDOG_TIMEOUT_NS 2000000000ULL

char** sitl_saved_argv;

// implemented in sitl_it.c
extern void sitl_irq_handler(int irq);
// implemented in sys_can_SITL.c when DroneCAN is compiled in
void sitl_can_poll(void) __attribute__((weak));
void sitl_can_poll(void) { }

uint64_t sitl_time_ns(void)
{
    return sim_time_ns_v;
}

bool sitl_in_sim_thread(void)
{
    return pthread_equal(pthread_self(), sim_thread_id);
}

static uint64_t wallclock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

/*
  NVIC emulation
 */
void sitl_nvic_set_priority(int irq, uint32_t prio)
{
    irq_prio[irq] = prio;
}

void sitl_nvic_enable_irq(int irq)
{
    __atomic_fetch_or(&irq_enabled, 1U << irq, __ATOMIC_SEQ_CST);
}

void sitl_nvic_disable_irq(int irq)
{
    __atomic_fetch_and(&irq_enabled, ~(1U << irq), __ATOMIC_SEQ_CST);
}

/*
  dispatch block diagnostic: records why a pending COMP/COM interrupt is
  not being delivered, classified per simulation step, reported when the
  delivery latency exceeds 20us (bounded number of reports)
 */
static uint64_t irq_pend_ns[SITL_IRQ_MAX];
static int current_irq = -1;
static struct {
    uint32_t steps_primask;
    uint32_t steps_active[SITL_IRQ_MAX];
    uint32_t steps_disabled;
    uint32_t steps_free;
} blocked;

void sitl_irq_pend(int irq)
{
    const uint32_t old = __atomic_fetch_or(&irq_pending, 1U << irq, __ATOMIC_SEQ_CST);
    if ((old & (1U << irq)) == 0) {
        irq_pend_ns[irq] = sim_time_ns_v;
    }
}

void sitl_primask_set(void)
{
    // plain atomic store: the dispatcher re-checks primask after parking
    // the firmware thread, so no lock is needed. Critical sections are
    // extremely frequent (micros64 runs one per call) and must be cheap
    __atomic_store_n(&primask, 1, __ATOMIC_SEQ_CST);
}

void sitl_primask_clear(void)
{
    if (!sitl_in_sim_thread()) {
        // grants are per critical section: unconsumed ones must not
        // accumulate into a reservoir that lets the simulation run
        // through a later, host-stretched critical section
        __atomic_store_n(&fw_grant_ns, 0, __ATOMIC_SEQ_CST);
    }
    __atomic_store_n(&primask, 0, __ATOMIC_SEQ_CST);
}

/*
  firmware thread suspension. SIGUSR1 parks the firmware thread on
  resume_sem; sem_post/sem_wait are async-signal-safe
 */
static void sigusr1_handler(int sig)
{
    (void)sig;
    const int saved_errno = errno;
    sem_post(&park_sem);
    while (sem_wait(&resume_sem) == -1 && errno == EINTR) {
    }
    errno = saved_errno;
}

static bool suspend_firmware(void)
{
    pthread_kill(fw_thread_id, SIGUSR1);
    // a timed wait so a firmware thread that is exiting or execing
    // (NVIC_SystemReset) cannot deadlock the simulation
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += 2;
    for (;;) {
        if (sem_timedwait(&park_sem, &ts) == 0) {
            return true;
        }
        if (errno == EINTR) {
            continue;
        }
        return false;
    }
}

static void resume_firmware(void)
{
    sem_post(&resume_sem);
}

// priority of the currently executing handler, NVIC style (lower value
// is higher priority). 1000 = thread level, nothing active
static int active_irq_prio = 1000;


// run any pending interrupt with higher priority than the active one.
// Called from the dispatch loop and re-entrantly from sitl_isr_read_tick
// so a long running handler (eg tenKhzRoutine busy waiting on a timer)
// can be preempted by the comparator, as the NVIC does on real hardware
static void run_pending_irqs(void)
{
    for (;;) {
        if (primask) {
            return;
        }
        const uint32_t active = irq_pending & irq_enabled;
        if (active == 0) {
            return;
        }
        int best = -1;
        for (int irq = 0; irq < SITL_IRQ_MAX; irq++) {
            if ((active & (1U << irq)) == 0) {
                continue;
            }
            if (best < 0 || irq_prio[irq] < irq_prio[best]) {
                best = irq;
            }
        }
        if (irq_prio[best] >= active_irq_prio) {
            // equal or lower priority does not preempt
            return;
        }
        __atomic_fetch_and(&irq_pending, ~(1U << best), __ATOMIC_SEQ_CST);
        if (best == SITL_IRQ_COMP || best == SITL_IRQ_COM) {
            const uint64_t lat = sim_time_ns_v - irq_pend_ns[best];
            static int prints;
            if (lat > 20000 && prints < 12) {
                prints++;
                fprintf(stderr,
                    "SITL: irq %d blocked %.1fus at t=%.4f: primask=%u dis=%u free=%u"
                    " act=[%u,%u,%u,%u,%u,%u]\n",
                    best, lat * 1e-3, sim_time_ns_v * 1e-9,
                    blocked.steps_primask, blocked.steps_disabled, blocked.steps_free,
                    blocked.steps_active[0], blocked.steps_active[1],
                    blocked.steps_active[2], blocked.steps_active[3],
                    blocked.steps_active[4], blocked.steps_active[5]);
            }
            memset(&blocked, 0, sizeof(blocked));
        }
        const int saved_prio = active_irq_prio;
        const int saved_irq = current_irq;
        active_irq_prio = irq_prio[best];
        current_irq = best;
        sitl_irq_handler(best);
        current_irq = saved_irq;
        active_irq_prio = saved_prio;
    }
}

/*
  deliver pending enabled interrupts, called from the sim thread between
  physics steps
 */
static void sitl_dispatch(void)
{
    if ((irq_pending & irq_enabled) == 0 || primask) {
        return;
    }
    if (!suspend_firmware()) {
        return;
    }
    // the firmware may have entered a critical section between our check
    // and it parking; if so it is now parked inside the section and we
    // must not run handlers (an IRQ arriving just after cpsid is held
    // pending on real hardware too)
    if (!primask) {
        run_pending_irqs();
    }
    resume_firmware();
}

/*
  watchdog
 */
void sitl_watchdog_reload(void)
{
    extern void motor_log_mainloop(void);
    motor_log_mainloop();
    watchdog_last_reload_ns = sim_time_ns_v;
    if (!sitl_in_sim_thread() && sitl_cfg.sim.loop_time_ns > 0 && sitl_cfg.speedup > 0) {
        // approximate the real main loop execution time so the firmware
        // thread does not spin flat out
        const uint64_t delay_ns = (uint64_t)(sitl_cfg.sim.loop_time_ns / sitl_cfg.speedup);
        if (sitl_cfg.nosleep) {
            const uint64_t deadline = wallclock_ns() + delay_ns;
            while (wallclock_ns() < deadline) {
            }
        } else {
            struct timespec ts = { 0, (long)delay_ns };
            nanosleep(&ts, NULL);
        }
    }
}

void sitl_watchdog_enable(void)
{
    watchdog_last_reload_ns = sim_time_ns_v;
    watchdog_running = sitl_cfg.sim.watchdog_enabled;
}

static void watchdog_check(void)
{
    if (watchdog_running && sim_time_ns_v - watchdog_last_reload_ns > WATCHDOG_TIMEOUT_NS) {
        fprintf(stderr, "SITL: watchdog reset at t=%.3fs\n", sim_time_ns_v * 1.0e-9);
        sitl_system_reset();
    }
}

void sitl_system_reset(void)
{
    // block the suspension signal so a concurrent interrupt delivery
    // cannot park this thread on the way into exec
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGUSR1);
    pthread_sigmask(SIG_BLOCK, &set, NULL);
    fprintf(stderr, "SITL: reset at t=%.3fs\n", sim_time_ns_v * 1.0e-9);
    execv("/proc/self/exe", sitl_saved_argv);
    fprintf(stderr, "SITL: execv failed: %s\n", strerror(errno));
    _exit(1);
}

/*
  advance simulation by one physics step. Called from the sim thread main
  loop and re-entrantly from interrupt handlers that busy wait on time
  (delayMicros, comparator filter reads)
 */
static void sim_step_once(void)
{
    const uint32_t dt = sitl_cfg.sim.physics_dt_ns;
    motor_step(sim_time_ns_v, dt);
    sim_time_ns_v += dt;
    sitl_timers_step(sim_time_ns_v);
    sitl_state_step(sim_time_ns_v);

    // dispatch block tracer: classify this step if a priority 0 interrupt
    // has been pending for a while
    for (int irq = SITL_IRQ_COMP; irq <= SITL_IRQ_COM; irq++) {
        if ((irq_pending & (1U << irq)) == 0) {
            continue;
        }
        if (sim_time_ns_v - irq_pend_ns[irq] <= 20000) {
            continue;
        }
        if (current_irq >= 0) {
            // a handler is executing (possibly holding primask itself)
            blocked.steps_active[current_irq]++;
        } else if (primask) {
            blocked.steps_primask++;
        } else if ((irq_enabled & (1U << irq)) == 0) {
            blocked.steps_disabled++;
        } else {
            blocked.steps_free++;
        }
        break;
    }
}

void sitl_step_from_isr(void)
{
    sim_step_once();
}

void sitl_fw_read_tick(void)
{
    if (primask && !sitl_in_sim_thread()) {
        __atomic_fetch_add(&fw_grant_ns, sitl_cfg.sim.isr_read_ns, __ATOMIC_SEQ_CST);
    }
}

void sitl_isr_read_tick(void)
{
    // only called from the sim thread (interrupt context), no locking
    // needed. Advancing a full physics step per register read would make
    // handler busy loops take ~10x longer in simulated time than on real
    // hardware, which breaks the comparator filter in interruptRoutine()
    static uint32_t accum_ns;
    accum_ns += sitl_cfg.sim.isr_read_ns;
    while (accum_ns >= sitl_cfg.sim.physics_dt_ns) {
        accum_ns -= sitl_cfg.sim.physics_dt_ns;
        sim_step_once();
        // let higher priority interrupts preempt the current handler
        run_pending_irqs();
    }
}

// try to switch the calling thread to SCHED_FIFO, warning on failure
static void set_realtime(const char* what)
{
    if (!sitl_cfg.realtime) {
        return;
    }
    struct sched_param sp = { .sched_priority = 50 };
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
        fprintf(stderr, "SITL: SCHED_FIFO failed for %s thread: %s\n",
            what, strerror(errno));
        return;
    }
    // with --nosleep the threads never block, so default RT throttling
    // (kernel.sched_rt_runtime_us=950000) would stall them 50ms/second
    if (sitl_cfg.nosleep) {
        FILE* f = fopen("/proc/sys/kernel/sched_rt_runtime_us", "r");
        if (f) {
            long v = 0;
            if (fscanf(f, "%ld", &v) == 1 && v != -1) {
                fprintf(stderr,
                    "SITL: WARNING: RT throttling is enabled and will stall "
                    "--nosleep --realtime; run "
                    "'sysctl -w kernel.sched_rt_runtime_us=-1'\n");
            }
            fclose(f);
        }
    }
    fprintf(stderr, "SITL: %s thread using SCHED_FIFO\n", what);
}

static void* sim_thread_main(void* arg)
{
    (void)arg;
    set_realtime("sim");
#ifdef __linux__
    // timer slack defaults to 50us which ruins short pacing sleeps
    prctl(PR_SET_TIMERSLACK, 1UL);
#endif
    const uint64_t wall0 = wallclock_ns();
    uint64_t next_can_poll_ns = 0;
    uint64_t next_pace_check_ns = 0;
    uint64_t verbose_last_ns = 0;
    uint64_t verbose_last_wall = wall0;
    uint64_t pace_wall_ref = wall0;
    uint64_t pace_sim_ref = 0;
    float pace_speedup = sitl_cfg.speedup;

    uint32_t grant_accum_ns = 0;
    for (;;) {
        /*
          while the firmware thread holds PRIMASK outside of interrupt
          context, simulated time may only advance as granted by the
          firmware's own timer reads. On the real MCU a critical section
          lasts nanoseconds; if the host deschedules the firmware thread
          inside one (OS preemption, hard/soft irqs on its core), a free
          running clock would block interrupt delivery for hundreds of
          microseconds of simulated time and lose commutation timing.
          current_irq is only ever set by this thread, so the check does
          not race: ISR-context PRIMASK always has current_irq >= 0
         */
        if (primask && current_irq < 0) {
            const uint64_t g = fw_grant_ns;
            if (g > 0) {
                __atomic_fetch_sub(&fw_grant_ns, g, __ATOMIC_SEQ_CST);
                grant_accum_ns += (uint32_t)g;
            }
            if (grant_accum_ns < sitl_cfg.sim.physics_dt_ns) {
                continue;
            }
            grant_accum_ns -= sitl_cfg.sim.physics_dt_ns;
        } else {
            grant_accum_ns = 0;
        }
        sim_step_once();
        const uint64_t now = sim_time_ns_v;

        if (now >= next_can_poll_ns) {
            next_can_poll_ns = now + 100000; // 100us
            sitl_can_poll();
            sitl_input_poll();
            sitl_state_poll();
        }
        watchdog_check();
        sitl_dispatch();

        // pace simulated time against the wall clock, sleeping to an
        // absolute deadline so overshoot does not accumulate. The
        // references rebase when the speedup changes at runtime (GUI
        // slow motion control) so the mapping stays continuous
        if (sitl_cfg.speedup != pace_speedup) {
            pace_speedup = sitl_cfg.speedup;
            pace_wall_ref = wallclock_ns();
            pace_sim_ref = now;
        }
        if (pace_speedup > 0 && now >= next_pace_check_ns) {
            next_pace_check_ns = now + 50000; // check every 50us of sim time
            const uint64_t target_wall = pace_wall_ref + (uint64_t)((double)(now - pace_sim_ref) / pace_speedup);
            const uint64_t wall = wallclock_ns();
            if (sitl_cfg.nosleep) {
                while (wallclock_ns() < target_wall) {
                }
            } else if (target_wall > wall + 100000) {
                struct timespec ts = { target_wall / 1000000000ULL, target_wall % 1000000000ULL };
                clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
            }
        }

        if (sitl_cfg.verbose && now - verbose_last_ns >= 1000000000ULL) {
            const uint64_t wall = wallclock_ns();
            const float ratio = (float)(now - verbose_last_ns) / (float)(wall - verbose_last_wall);
            verbose_last_ns = now;
            verbose_last_wall = wall;
            motor_print_state(now, ratio);
        }
    }
    return NULL;
}

void sitl_start_sim_thread(void)
{
    fw_thread_id = pthread_self();
    set_realtime("firmware");
#ifdef __linux__
    // timer slack is per thread and defaults to 50us
    prctl(PR_SET_TIMERSLACK, 1UL);
#endif
    sem_init(&park_sem, 0, 0);
    sem_init(&resume_sem, 0, 0);

    // sitl_system_reset blocks SIGUSR1 on the way into execv; both the
    // mask and a possibly pending SIGUSR1 survive exec. Discard any
    // pending instance and unblock before installing the real handler
    signal(SIGUSR1, SIG_IGN);
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGUSR1);
    pthread_sigmask(SIG_UNBLOCK, &set, NULL);

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = sigusr1_handler;
    sa.sa_flags = SA_RESTART;
    sigaction(SIGUSR1, &sa, NULL);

    pthread_create(&sim_thread_id, NULL, sim_thread_main, NULL);
}
