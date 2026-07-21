/*
 * zc_handoff.h - quality-based open-loop <-> closed-loop handoff
 *
 * Extends the legacy hard commutation_interval vs polling_mode_changeover
 * test for the normal (non stall-protection) path. Poll mode notes intervals
 * at each found ZC; closed-loop notes intervals from the commutation path.
 *
 * Enter: legacy fast CI (no ZC minimum), or stable poll-interval CV after
 *        min ZC count (low-KV early enter).
 * Exit:  CI_ABS_MAX rail; at/below legacy (changeover+500) never exit on
 *        quality; above that, drop unless CV still excellent (low-KV hold).
 * Confirm filter rejects are NOT used for exit — normal multi-sample noise.
 *
 * Call zcHandoffReset() on stall/desync/forced open-loop so the ring does
 * not survive a restart (stall-protection enter never notes poll intervals).
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

/* Non-zero => enter interrupt closed-loop (old_routine = 0). */
uint8_t zcHandoffShouldEnterClosedLoop(uint32_t commutation_interval);

/* Non-zero => drop back to poll open-loop (old_routine = 1). */
uint8_t zcHandoffShouldExitClosedLoop(uint32_t average_interval);

#endif /* ZC_HANDOFF_H_ */
