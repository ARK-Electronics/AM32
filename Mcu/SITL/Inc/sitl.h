/*
  sitl.h - internal API of the AM32 SITL runtime

  The firmware runs unmodified in one thread ("firmware thread") while a
  second thread ("sim thread") owns simulated time, the motor physics and
  delivery of emulated interrupts. Interrupt handlers always execute in the
  sim thread with the firmware thread suspended, giving the same
  run-to-completion semantics as a real MCU.

  All state shared between the two threads is either written through the
  helpers here or is a plain volatile word. x86 total-store-ordering is
  assumed.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

// emulated interrupt sources, in NVIC style. Default priorities are set to
// match the g431 target, main.c re-prioritises at runtime
enum sitl_irq {
    SITL_IRQ_COMP = 0, // BEMF comparator EXTI
    SITL_IRQ_COM, // commutation timer (TIM16) update
    SITL_IRQ_TENKHZ, // 20kHz loop timer (TIM6)
    SITL_IRQ_DMA, // input capture DMA (unused, dshot not supported)
    SITL_IRQ_EXTI15, // software interrupt for dshot (unused)
    SITL_IRQ_CAN, // CAN frame RX poll
    SITL_IRQ_MAX
};

// simulated monotonic time since start
uint64_t sitl_time_ns(void);

// true when called from the sim thread (i.e. from interrupt context)
bool sitl_in_sim_thread(void);

// advance the simulation by one physics step. Only legal from the sim
// thread; used so that busy loops inside interrupt handlers (delayMicros,
// comparator filter re-reads) still see time advance
void sitl_step_from_isr(void);

// account for one register/comparator read from interrupt context. Each
// read costs sim.isr_read_ns (about what a peripheral read costs on the
// real MCU); whole physics steps run once enough time has accumulated
void sitl_isr_read_tick(void);

// account for a register read from the firmware thread; grants simulated
// time while the firmware holds PRIMASK
void sitl_fw_read_tick(void);


// NVIC emulation
void sitl_nvic_set_priority(int irq, uint32_t prio);
void sitl_nvic_enable_irq(int irq);
void sitl_nvic_disable_irq(int irq);
void sitl_irq_pend(int irq);
void sitl_primask_set(void); // __disable_irq
void sitl_primask_clear(void); // __enable_irq
void sitl_system_reset(void) __attribute__((noreturn));

// watchdog
void sitl_watchdog_reload(void);
void sitl_watchdog_enable(void);

// timers (see sitl_timers.c)
enum sitl_tim_idx {
    SITL_TIM1_IDX = 0, // PWM timer, 160MHz, ARR/CCR preloaded
    SITL_TIM2_IDX, // INTERVAL_TIMER, 2MHz, 32 bit
    SITL_TIM6_IDX, // TEN_KHZ_TIMER, 1MHz, periodic update
    SITL_TIM16_IDX, // COM_TIMER, 2MHz, update interrupt
    SITL_TIM17_IDX, // UTILITY_TIMER, 1MHz, 16 bit free running
    SITL_NUM_TIMS
};

typedef struct {
    volatile uint32_t CNT;
    volatile uint32_t ARR;
    volatile uint32_t PSC;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t BDTR;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
} SITL_TIM_TypeDef;

// dereference an emulated timer, syncing CNT from simulated time
SITL_TIM_TypeDef* sitl_tim_deref(int idx);

uint32_t sitl_interval_timer_count(void);
void sitl_interval_timer_set(uint32_t cnt);
void sitl_com_int_arm(uint32_t time);
void sitl_com_int_disable(void);
void sitl_com_int_enable(void);
void sitl_tim1_set_duty_all(uint16_t duty);
void sitl_tim1_set_duty(int chan, uint16_t duty);
void sitl_tim1_set_psc(uint16_t psc);
void sitl_tim1_set_arr(uint16_t arr);
void sitl_tim1_force_update(void);
void sitl_tenkhz_enable(void);

// called by the sim thread each physics step to check timer events
void sitl_timers_step(uint64_t now_ns);
void sitl_timers_init(void);

// PWM output sampling for the physics (phase 0..2), true when the high
// side compare is active for the current TIM1 counter phase
bool sitl_tim1_pwm_out(int chan, uint64_t now_ns);

// EXTI emulation for the comparator lines
#define SITL_EXTI_LINE_21 (1UL << 21)
#define SITL_EXTI_LINE_22 (1UL << 22)

typedef struct {
    volatile uint32_t IMR;
    volatile uint32_t RTSR;
    volatile uint32_t FTSR;
    volatile uint32_t PR;
} sitl_exti_t;

extern sitl_exti_t sitl_exti;

// bridge output modes per phase, set by phaseouts.c, read by the physics
enum sitl_phase_mode {
    SITL_PHASE_FLOAT = 0, // both fets off
    SITL_PHASE_LOW, // low side on
    SITL_PHASE_PWM, // high side pwm, low side complementary if comp_pwm
    SITL_PHASE_PWM_NOCOMP, // high side pwm, low side off
    SITL_PHASE_BRAKE_PWM, // low side driven by complementary pwm
};

extern volatile uint8_t sitl_phase_mode[3];

// comparator state: which phase is floating and the latched output
extern volatile uint8_t sitl_comp_phase; // 0=A 1=B 2=C
extern volatile uint8_t sitl_comp_out;

// sensor snapshot from the physics, seqlock protected
typedef struct {
    float bus_voltage; // V at the ESC input
    float bus_current; // A into the bridge
    float temperature_c;
    float rpm; // mechanical, signed
} sitl_sensors_t;

void sitl_sensors_read(sitl_sensors_t* out);
void sitl_sensors_write(const sitl_sensors_t* in); // sim thread only

// lifecycle
void sitl_start_sim_thread(void);
extern char** sitl_saved_argv;
