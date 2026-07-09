/*
 * esc_state.c - top-level ESC drive state machine
 */

#include "esc_state.h"

#include "motor_runtime.h"
#include "common.h"
#include "signal.h"
#include "eeprom.h"

volatile esc_state_t esc_state = ESC_DISARMED;

/* stuck-rotor latch value used by setInput / faultHandleStuckRotorIfNeeded */
#define ESC_STUCK_LATCH 102

const char *escStateName(esc_state_t s)
{
    switch (s) {
    case ESC_DISARMED:     return "DISARMED";
    case ESC_ARMING:       return "ARMING";
    case ESC_ARMED_IDLE:   return "ARMED_IDLE";
    case ESC_SINE_START:   return "SINE_START";
    case ESC_OPEN_LOOP:    return "OPEN_LOOP";
    case ESC_CLOSED_LOOP:  return "CLOSED_LOOP";
    case ESC_BRAKE:        return "BRAKE";
    case ESC_FAULT_STUCK:  return "FAULT_STUCK";
    case ESC_FAULT_SIGNAL: return "FAULT_SIGNAL";
    case ESC_FAULT_LVC:    return "FAULT_LVC";
    default:               return "?";
    }
}

esc_state_t escGetState(void)
{
    return esc_state;
}

uint8_t escIsFault(void)
{
    return (uint8_t)(esc_state >= ESC_FAULT_STUCK);
}

uint8_t escIsDriving(void)
{
    return (uint8_t)(esc_state == ESC_SINE_START
        || esc_state == ESC_OPEN_LOOP
        || esc_state == ESC_CLOSED_LOOP);
}

void escReconcileFromFlags(void)
{
    /* Latched faults win over drive mode. */
    if (bemf_timeout_happened == ESC_STUCK_LATCH) {
        esc_state = ESC_FAULT_STUCK;
        return;
    }
    if (LOW_VOLTAGE_CUTOFF) {
        esc_state = ESC_FAULT_LVC;
        return;
    }

    if (!armed) {
        if (inputSet) {
            esc_state = ESC_ARMING;
        } else {
            esc_state = ESC_DISARMED;
        }
        return;
    }

    /* Armed */
    if (stepper_sine) {
        esc_state = ESC_SINE_START;
        return;
    }
    if (prop_brake_active && !running) {
        esc_state = ESC_BRAKE;
        return;
    }
    if (running) {
        esc_state = old_routine ? ESC_OPEN_LOOP : ESC_CLOSED_LOOP;
        return;
    }
    esc_state = ESC_ARMED_IDLE;
}

void escToDisarmed(void)
{
    armed = 0;
    running = 0;
    stepper_sine = 0;
    esc_state = ESC_DISARMED;
}

void escToArming(void)
{
    /* inputSet already true when arming; not armed yet */
    armed = 0;
    esc_state = ESC_ARMING;
}

void escToArmedIdle(void)
{
    armed = 1;
    running = 0;
    stepper_sine = 0;
    esc_state = ESC_ARMED_IDLE;
}

void escToSineStart(void)
{
    armed = 1;
    stepper_sine = 1;
    esc_state = ESC_SINE_START;
}

void escToOpenLoop(void)
{
    armed = 1;
    running = 1;
    old_routine = 1;
    stepper_sine = 0;
    esc_state = ESC_OPEN_LOOP;
}

void escToClosedLoop(void)
{
    armed = 1;
    running = 1;
    old_routine = 0;
    stepper_sine = 0;
    esc_state = ESC_CLOSED_LOOP;
}

void escToBrake(void)
{
    armed = 1;
    prop_brake_active = 1;
    esc_state = ESC_BRAKE;
}

void escToFaultStuck(void)
{
    input = 0;
    bemf_timeout_happened = ESC_STUCK_LATCH;
    running = 0;
    stepper_sine = 0;
    esc_state = ESC_FAULT_STUCK;
}

void escToFaultSignal(void)
{
    armed = 0;
    input = 0;
    inputSet = 0;
    running = 0;
    stepper_sine = 0;
    esc_state = ESC_FAULT_SIGNAL;
}

void escToFaultLvc(void)
{
    LOW_VOLTAGE_CUTOFF = 1;
    input = 0;
    running = 0;
    armed = 0;
    stepper_sine = 0;
    esc_state = ESC_FAULT_LVC;
}

void escEnterRunningOpenLoop(void)
{
    armed = 1;
    running = 1;
    stepper_sine = 0;
    /* leave old_routine as-is unless still at default poll path */
    if (!old_routine) {
        /* startMotor path with interrupt ZC already enabled elsewhere */
        esc_state = ESC_CLOSED_LOOP;
    } else {
        esc_state = ESC_OPEN_LOOP;
    }
}

void escSineHandoffToOpenLoop(void)
{
    stepper_sine = 0;
    running = 1;
    old_routine = 1;
    prop_brake_active = 0;
    esc_state = ESC_OPEN_LOOP;
}

void escNoteStallOrDesync(uint8_t stop_if_low_throttle)
{
    old_routine = 1;
    if (stop_if_low_throttle && input < 48) {
        running = 0;
        commutation_interval = 5000;
        esc_state = armed ? ESC_ARMED_IDLE : ESC_DISARMED;
    } else if (running) {
        esc_state = ESC_OPEN_LOOP;
    } else if (armed) {
        esc_state = ESC_ARMED_IDLE;
    } else {
        esc_state = ESC_DISARMED;
    }
}
