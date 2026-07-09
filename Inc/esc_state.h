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
 */
#ifndef ESC_STATE_H_
#define ESC_STATE_H_

#include <stdint.h>

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
} esc_state_t;

extern volatile esc_state_t esc_state;

const char *escStateName(esc_state_t s);
esc_state_t escGetState(void);

/* True for stuck / signal / LVC latches. */
uint8_t escIsFault(void);
/* Sine, open-loop, or closed-loop drive. */
uint8_t escIsDriving(void);

/*
 * Rebuild esc_state from legacy flags. Call after ISR-side flag changes
 * (old_routine, running) or once per main-loop tick.
 */
void escReconcileFromFlags(void);

/* --- named transitions (set state + flags) --- */

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

/* Convenience: armed + running with old_routine poll path (startup). */
void escEnterRunningOpenLoop(void);
/* Sine handoff to six-step (old_routine poll, running). */
void escSineHandoffToOpenLoop(void);
/* Stall / desync: back to poll path; optionally stop if throttle low. */
void escNoteStallOrDesync(uint8_t stop_if_low_throttle);

#endif /* ESC_STATE_H_ */
