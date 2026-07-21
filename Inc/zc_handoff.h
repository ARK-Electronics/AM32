/*
 * zc_handoff.h - quality-based open-loop <-> closed-loop handoff
 *
 * Replaces sole reliance on a hard commutation_interval threshold for the
 * normal (non stall-protection) path. Poll mode notes intervals at each
 * found ZC; closed-loop notes intervals from the commutation ISR path and
 * confirm reject/accept from the comparator ISR.
 *
 * Safety rails only: refuse CL when impossibly slow (CI_ABS_MAX), require a
 * minimum zero_cross count, and use CV / confirm-reject hysteresis so the
 * mode does not chatter.
 */
#ifndef ZC_HANDOFF_H_
#define ZC_HANDOFF_H_

#include <stdint.h>

/* 0.5 us INTERVAL_TIMER ticks: refuse closed-loop if this slow (near stop). */
#ifndef ZC_HANDOFF_CI_ABS_MAX
#	define ZC_HANDOFF_CI_ABS_MAX 12000u
#endif

void zcHandoffReset(void);

/* Call after each poll-mode ZC updates commutation_interval. */
void zcHandoffNotePollInterval(uint32_t commutation_interval);

/* Call from closed-loop path when a new interval is known. */
void zcHandoffNoteClosedInterval(uint32_t commutation_interval);

/* Comparator confirm outcome (interrupt path). */
void zcHandoffNoteConfirmReject(void);
void zcHandoffNoteConfirmAccept(void);

/* Non-zero => enter interrupt closed-loop (old_routine = 0). */
uint8_t zcHandoffShouldEnterClosedLoop(uint32_t commutation_interval);

/* Non-zero => drop back to poll open-loop (old_routine = 1). */
uint8_t zcHandoffShouldExitClosedLoop(uint32_t average_interval);

#endif /* ZC_HANDOFF_H_ */
