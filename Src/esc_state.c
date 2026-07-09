/*
 * esc_state.c - top-level ESC drive state machine
 */

#include "esc_state.h"

#include "motor_runtime.h"
#include "common.h"
#include "signal.h"
#include "eeprom.h"

volatile esc_state_t esc_state = ESC_DISARMED;
volatile uint16_t esc_illegal_edge_count = 0;

/* stuck-rotor latch value used by setInput / faultHandleStuckRotorIfNeeded */
#define ESC_STUCK_LATCH 102

/*
 * Allowed named-transition edges: bit N set => may enter esc_state_t N.
 * Self-transitions are always treated as allowed in escTransitionAllowed().
 * escReconcileFromFlags() forces state and does not consult this table.
 *
 * Keep in sync with hwci/tests/test_esc_state.py (ESC_ALLOWED).
 */
static const uint16_t esc_allowed[ESC_STATE_COUNT] = {
    /* DISARMED */
    [ESC_DISARMED] = (1u << ESC_DISARMED) | (1u << ESC_ARMING)
        | (1u << ESC_ARMED_IDLE) | (1u << ESC_FAULT_SIGNAL) | (1u << ESC_FAULT_LVC),
    /* ARMING */
    [ESC_ARMING] = (1u << ESC_ARMING) | (1u << ESC_DISARMED) | (1u << ESC_ARMED_IDLE)
        | (1u << ESC_FAULT_SIGNAL) | (1u << ESC_FAULT_LVC),
    /* ARMED_IDLE */
    [ESC_ARMED_IDLE] = (1u << ESC_ARMED_IDLE) | (1u << ESC_SINE_START)
        | (1u << ESC_OPEN_LOOP) | (1u << ESC_CLOSED_LOOP) | (1u << ESC_BRAKE)
        | (1u << ESC_DISARMED) | (1u << ESC_FAULT_STUCK) | (1u << ESC_FAULT_SIGNAL)
        | (1u << ESC_FAULT_LVC),
    /* SINE_START */
    [ESC_SINE_START] = (1u << ESC_SINE_START) | (1u << ESC_OPEN_LOOP)
        | (1u << ESC_ARMED_IDLE) | (1u << ESC_BRAKE) | (1u << ESC_DISARMED)
        | (1u << ESC_FAULT_STUCK) | (1u << ESC_FAULT_SIGNAL) | (1u << ESC_FAULT_LVC),
    /* OPEN_LOOP */
    [ESC_OPEN_LOOP] = (1u << ESC_OPEN_LOOP) | (1u << ESC_CLOSED_LOOP)
        | (1u << ESC_ARMED_IDLE) | (1u << ESC_BRAKE) | (1u << ESC_SINE_START)
        | (1u << ESC_DISARMED) | (1u << ESC_FAULT_STUCK) | (1u << ESC_FAULT_SIGNAL)
        | (1u << ESC_FAULT_LVC),
    /* CLOSED_LOOP */
    [ESC_CLOSED_LOOP] = (1u << ESC_CLOSED_LOOP) | (1u << ESC_OPEN_LOOP)
        | (1u << ESC_ARMED_IDLE) | (1u << ESC_BRAKE) | (1u << ESC_DISARMED)
        | (1u << ESC_FAULT_STUCK) | (1u << ESC_FAULT_SIGNAL) | (1u << ESC_FAULT_LVC),
    /* BRAKE */
    [ESC_BRAKE] = (1u << ESC_BRAKE) | (1u << ESC_ARMED_IDLE) | (1u << ESC_OPEN_LOOP)
        | (1u << ESC_SINE_START) | (1u << ESC_DISARMED) | (1u << ESC_FAULT_STUCK)
        | (1u << ESC_FAULT_SIGNAL) | (1u << ESC_FAULT_LVC),
    /* FAULT_STUCK: latch; clear only via reconcile when latch drops */
    [ESC_FAULT_STUCK] = (1u << ESC_FAULT_STUCK) | (1u << ESC_ARMED_IDLE)
        | (1u << ESC_DISARMED) | (1u << ESC_FAULT_SIGNAL) | (1u << ESC_FAULT_LVC),
    /* FAULT_SIGNAL: expect reset */
    [ESC_FAULT_SIGNAL] = (1u << ESC_FAULT_SIGNAL) | (1u << ESC_DISARMED),
    /* FAULT_LVC: latched until power cycle */
    [ESC_FAULT_LVC] = (1u << ESC_FAULT_LVC),
};

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

uint8_t escTransitionAllowed(esc_state_t from, esc_state_t to)
{
    if ((unsigned)from >= ESC_STATE_COUNT || (unsigned)to >= ESC_STATE_COUNT) {
        return 0;
    }
    if (from == to) {
        return 1;
    }
    return (uint8_t)((esc_allowed[from] >> to) & 1u);
}

/*
 * Commit a named-transition target. Illegal edges still apply (never brick
 * the motor on a table bug) but increment esc_illegal_edge_count. Define
 * ESC_STATE_STRICT to hang on illegal edges in debug builds.
 */
static void escCommitState(esc_state_t next)
{
    esc_state_t from = esc_state;
    if (from != next && !escTransitionAllowed(from, next)) {
        if (esc_illegal_edge_count < 0xFFFFu) {
            esc_illegal_edge_count++;
        }
#ifdef ESC_STATE_STRICT
        while (1) {
            /* illegal edge: inspect from/next in debugger */
        }
#endif
    }
    esc_state = next;
}

/* Force without edge check (reconcile / recovery). */
static void escForceState(esc_state_t next)
{
    esc_state = next;
}

uint8_t escIsFault(void)
{
    return (uint8_t)(esc_state >= ESC_FAULT_STUCK && esc_state < ESC_STATE_COUNT);
}

/*
 * Policy predicates used from the 20 kHz path must follow the legacy flags
 * (ISR may flip old_routine/running before the next reconcile). Named
 * transitions and escReconcileFromFlags() keep esc_state aligned for host
 * logs and main-loop policy.
 */
uint8_t escIsArmed(void)
{
    return (uint8_t)(armed != 0);
}

uint8_t escIsDriving(void)
{
    return (uint8_t)(running != 0 || stepper_sine != 0);
}

uint8_t escInSineStart(void)
{
    return (uint8_t)(stepper_sine != 0);
}

uint8_t escInOpenLoop(void)
{
    return (uint8_t)(running != 0 && old_routine != 0);
}

uint8_t escInClosedLoop(void)
{
    return (uint8_t)(running != 0 && old_routine == 0 && !stepper_sine);
}

uint8_t escInBrake(void)
{
    return (uint8_t)(prop_brake_active != 0 && running == 0);
}

uint8_t escMaySixStepThrottle(void)
{
    return (uint8_t)(armed != 0 && stepper_sine == 0);
}

uint8_t escInPollZcDrive(void)
{
    /* Exact legacy condition: old_routine && running */
    return (uint8_t)(old_routine != 0 && running != 0);
}

void escReconcileFromFlags(void)
{
    /* Latched faults win over drive mode. */
    if (bemf_timeout_happened == ESC_STUCK_LATCH) {
        escForceState(ESC_FAULT_STUCK);
        return;
    }
    if (LOW_VOLTAGE_CUTOFF) {
        escForceState(ESC_FAULT_LVC);
        return;
    }

    if (!armed) {
        if (inputSet) {
            escForceState(ESC_ARMING);
        } else {
            escForceState(ESC_DISARMED);
        }
        return;
    }

    /* Armed */
    if (stepper_sine) {
        escForceState(ESC_SINE_START);
        return;
    }
    if (prop_brake_active && !running) {
        escForceState(ESC_BRAKE);
        return;
    }
    if (running) {
        escForceState(old_routine ? ESC_OPEN_LOOP : ESC_CLOSED_LOOP);
        return;
    }
    escForceState(ESC_ARMED_IDLE);
}

void escToDisarmed(void)
{
    armed = 0;
    running = 0;
    stepper_sine = 0;
    escCommitState(ESC_DISARMED);
}

void escToArming(void)
{
    armed = 0;
    escCommitState(ESC_ARMING);
}

void escToArmedIdle(void)
{
    armed = 1;
    running = 0;
    stepper_sine = 0;
    escCommitState(ESC_ARMED_IDLE);
}

void escToSineStart(void)
{
    armed = 1;
    stepper_sine = 1;
    escCommitState(ESC_SINE_START);
}

void escToOpenLoop(void)
{
    armed = 1;
    running = 1;
    old_routine = 1;
    stepper_sine = 0;
    escCommitState(ESC_OPEN_LOOP);
}

void escToClosedLoop(void)
{
    armed = 1;
    running = 1;
    old_routine = 0;
    stepper_sine = 0;
    escCommitState(ESC_CLOSED_LOOP);
}

void escToBrake(void)
{
    armed = 1;
    prop_brake_active = 1;
    escCommitState(ESC_BRAKE);
}

void escToFaultStuck(void)
{
    input = 0;
    bemf_timeout_happened = ESC_STUCK_LATCH;
    running = 0;
    stepper_sine = 0;
    escCommitState(ESC_FAULT_STUCK);
}

void escToFaultSignal(void)
{
    armed = 0;
    input = 0;
    inputSet = 0;
    running = 0;
    stepper_sine = 0;
    escCommitState(ESC_FAULT_SIGNAL);
}

void escToFaultLvc(void)
{
    LOW_VOLTAGE_CUTOFF = 1;
    input = 0;
    running = 0;
    armed = 0;
    stepper_sine = 0;
    escCommitState(ESC_FAULT_LVC);
}

void escEnterRunningOpenLoop(void)
{
    armed = 1;
    running = 1;
    stepper_sine = 0;
    if (!old_routine) {
        escCommitState(ESC_CLOSED_LOOP);
    } else {
        escCommitState(ESC_OPEN_LOOP);
    }
}

void escSineHandoffToOpenLoop(void)
{
    stepper_sine = 0;
    running = 1;
    old_routine = 1;
    prop_brake_active = 0;
    escCommitState(ESC_OPEN_LOOP);
}

void escNoteStallOrDesync(uint8_t stop_if_low_throttle)
{
    old_routine = 1;
    if (stop_if_low_throttle && input < 48) {
        running = 0;
        commutation_interval = 5000;
        escCommitState(armed ? ESC_ARMED_IDLE : ESC_DISARMED);
    } else if (running) {
        escCommitState(ESC_OPEN_LOOP);
    } else if (armed) {
        escCommitState(ESC_ARMED_IDLE);
    } else {
        escCommitState(ESC_DISARMED);
    }
}
