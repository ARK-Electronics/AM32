/*
 * esc_state.h - top-level ESC drive state machine
 *
 * High-level policy mode for arming, run path, brake, and latched faults.
 * Legacy flags (armed, running, stepper_sine, old_routine, inputSet, …)
 * remain the hot-path surface for ISRs; this module keeps the enum in
 * sync and provides named transitions for multi-flag changes.
 *
 * Not used for per-commutation micro-steps (those stay in bemf_zc /
 * commutation).
 *
 * Predicates are static inline so RAM-resident callers (tenKhzRoutine)
 * do not pay long-call veneers into flash (F051 size/RAM win).
 */
#ifndef ESC_STATE_H_
#define ESC_STATE_H_

#include <stdint.h>
#include "motor_runtime.h"

typedef enum {
    ESC_DISARMED = 0,   /* waiting for protocol / arm sequence */
    ESC_ARMING,         /* valid input seen, zero-throttle arm timer */
    ESC_ARMED_IDLE,     /* armed, not driving (low throttle / stopped) */
    ESC_SINE_START,     /* stepper / sine soft-start path */
    ESC_OPEN_LOOP,      /* six-step running, poll ZC (old_routine) */
    ESC_CLOSED_LOOP,    /* six-step running, interrupt ZC */
    ESC_BRAKE,          /* prop / drag brake while not in closed drive */
    ESC_FAULT_STUCK,    /* stuck-rotor latch */
    ESC_FAULT_SIGNAL,   /* signal-loss path (typically then reset) */
    ESC_FAULT_LVC,      /* low-voltage cutoff latched */
    ESC_STATE_COUNT
} esc_state_t;

extern volatile esc_state_t esc_state;
/* Saturating count of named transitions that violated the edge table. */
extern volatile uint16_t esc_illegal_edge_count;

const char *escStateName(esc_state_t s);
esc_state_t escGetState(void);

/* --- predicates (flag-backed; ISR may update flags before reconcile) --- */

static inline uint8_t escIsFault(void)
{
    return (uint8_t)(esc_state >= ESC_FAULT_STUCK && esc_state < ESC_STATE_COUNT);
}

static inline uint8_t escIsArmed(void)
{
    return (uint8_t)(armed != 0);
}

static inline uint8_t escIsDriving(void)
{
    return (uint8_t)(running != 0 || stepper_sine != 0);
}

static inline uint8_t escInSineStart(void)
{
    return (uint8_t)(stepper_sine != 0);
}

static inline uint8_t escInOpenLoop(void)
{
    return (uint8_t)(running != 0 && old_routine != 0);
}

static inline uint8_t escInClosedLoop(void)
{
    return (uint8_t)(running != 0 && old_routine == 0 && !stepper_sine);
}

static inline uint8_t escInBrake(void)
{
    return (uint8_t)(prop_brake_active != 0 && running == 0);
}

static inline uint8_t escMaySixStepThrottle(void)
{
    return (uint8_t)(armed != 0 && stepper_sine == 0);
}

static inline uint8_t escInPollZcDrive(void)
{
    return (uint8_t)(old_routine != 0 && running != 0);
}

/* 1 if from->to is a legal named transition (self always legal). */
uint8_t escTransitionAllowed(esc_state_t from, esc_state_t to);

/*
 * Rebuild esc_state from legacy flags (force; bypasses edge table).
 * Call once per main-loop tick (runtimeMotorModeTick) / after multi-flag
 * transitions — not from RAM_FUNC 20 kHz path (flash veneer cost on F051).
 * Hot-path policy must use flag-backed predicates in this header.
 */
void escReconcileFromFlags(void);

/* --- named transitions (set flags + commit state via edge table) --- */

void escToDisarmed(void);
void escToArming(void);
void escToArmedIdle(void);
void escToSineStart(void);
void escToOpenLoop(void);
void escToClosedLoop(void);
void escToBrake(void);
void escToFaultStuck(void);
/* Disarm + clear inputSet; caller performs allOff / reset as needed. */
void escToFaultSignal(void);
void escToFaultLvc(void);

/* Convenience: armed + running with current old_routine path. */
void escEnterRunningOpenLoop(void);
/* Sine handoff to six-step (old_routine poll, running). */
void escSineHandoffToOpenLoop(void);
/* Stall / desync: back to poll path; optionally stop if throttle low. */
void escNoteStallOrDesync(uint8_t stop_if_low_throttle);

#endif /* ESC_STATE_H_ */
