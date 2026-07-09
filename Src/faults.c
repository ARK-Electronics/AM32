/*
 * faults.c - extracted from main/control_loop (behavior-neutral)
 */

#include "faults.h"

#include "main.h"
#include "common.h"
#include "motor_runtime.h"
#include "phaseouts.h"
#include "comparator.h"
#include "peripherals.h"
#include "functions.h"
#include "eeprom.h"
#include "signal.h"
#include "commutation.h"
#include "targets.h"
#include "esc_state.h"

#ifdef USE_RGB_LED
extern void setIndividualRGBLed(uint8_t, uint8_t, uint8_t);
#endif

extern volatile uint16_t zero_input_count;
extern volatile uint32_t dma_buffer[64];
extern void resetInputCaptureTimer(void);
extern char crawler_mode;

uint8_t faultHandleStuckRotorIfNeeded(void)
{
#ifndef BRUSHED_MODE
    if ((bemf_timeout_happened > bemf_timeout) && eepromBuffer.stuck_rotor_protection) {
        allOff();
        maskPhaseInterrupts();
        escToFaultStuck();
#ifdef USE_RGB_LED
        setIndividualRGBLed(1, 0, 0);
#endif
        return 1;
    }
#endif
    return 0;
}

void faultSignalTimeoutTick(void)
{
#if defined(FIXED_DUTY_MODE) || defined(FIXED_SPEED_MODE)
    if (getInputPinState()) {
        signaltimeout++;
        if (signaltimeout > LOOP_FREQUENCY_HZ) {
            NVIC_SystemReset();
        }
    } else {
        signaltimeout = 0;
    }
#else
    signaltimeout++;
#endif
}

void faultPollSignalTimeout(void)
{
    if (signaltimeout > (LOOP_FREQUENCY_HZ >> 1)) { // half second timeout when armed;
        if (armed) {
            allOff();
            escToFaultSignal();
            zero_input_count = 0;
            SET_DUTY_CYCLE_ALL(0);
            resetInputCaptureTimer();
            for (int i = 0; i < 64; i++) {
                dma_buffer[i] = 0;
            }
            NVIC_SystemReset();
        }
        if (signaltimeout > LOOP_FREQUENCY_HZ << 1) { // 2 second when not armed
            allOff();
            escToFaultSignal();
            zero_input_count = 0;
            SET_DUTY_CYCLE_ALL(0);
            resetInputCaptureTimer();
            for (int i = 0; i < 64; i++) {
                dma_buffer[i] = 0;
            }
            NVIC_SystemReset();
        }
    }
}

void faultUpdateBemfTimeoutPolicy(void)
{
#ifndef BRUSHED_MODE
    if ((zero_crosses > 1000) || (adjusted_input == 0)) {
        bemf_timeout_happened = 0;
    }
    if (zero_crosses > 100 && adjusted_input < 200) {
        bemf_timeout_happened = 0;
    }
    if (eepromBuffer.use_sine_start && adjusted_input < 160) {
        bemf_timeout_happened = 0;
    }

    if (crawler_mode) {
        if (adjusted_input < 400) {
            bemf_timeout_happened = 0;
        }
    } else {
        if (adjusted_input < 150) { // startup duty cycle should be low enough to not burn motor
            bemf_timeout = 100;
        } else {
            bemf_timeout = 10;
        }
    }
#endif
}

void faultHandleBemfIntervalStall(void)
{
    if (INTERVAL_TIMER_COUNT > 45000 && running == 1) {
        bemf_timeout_happened++;

        maskPhaseInterrupts();
        escNoteStallOrDesync(1);
        zero_crosses = 0;
        zcfoundroutine();
    }
}
