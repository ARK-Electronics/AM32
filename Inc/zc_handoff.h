/*
 * zc_handoff.h - quality-based open-loop <-> closed-loop handoff
 *
 * Extends the legacy hard commutation_interval vs polling_mode_changeover
 * test for the normal (non stall-protection) path.
 *
 * Enter:
 *   - Legacy: CI < changeover (same as pre-handoff; no ZC minimum).
 *   - Quality: CI near legacy band (changeover*2+slack), enough ZCs, low CV
 *     streak — low-KV early closed-loop only.
 *
 * Exit:
 *   - CI_ABS_MAX rail.
 *   - At/below changeover+500: never exit (legacy guarantee).
 *   - Above that: legacy-immediate exit unless this run was a quality enter
 *     still holding on good CV. (Legacy enter must NOT quality-hold — average
 *     lags CI during spool-up and would thrash OL↔CL.)
 *
 * Call zcHandoffOnEnter() after a successful enter, zcHandoffOnExit() when
 * dropping to poll, and zcHandoffReset()/clear on stall/desync paths.
 */
#ifndef ZC_HANDOFF_H_
#define ZC_HANDOFF_H_

#include <stdint.h>

/* 0.5 us INTERVAL_TIMER ticks: refuse closed-loop if this slow (near stop). */
#ifndef ZC_HANDOFF_CI_ABS_MAX
#	define ZC_HANDOFF_CI_ABS_MAX 12000u
#endif

/* zcHandoffShouldEnterClosedLoop() return values */
#define ZC_HANDOFF_ENTER_NONE 0u
#define ZC_HANDOFF_ENTER_LEGACY 1u
#define ZC_HANDOFF_ENTER_QUALITY 2u

void zcHandoffReset(void);

/* After enter decision: clears ring and arms quality-hold iff quality enter. */
void zcHandoffOnEnter(uint8_t enter_kind);
/* When dropping to poll from commutate exit path. */
void zcHandoffOnExit(void);

/* Call after each poll-mode ZC updates commutation_interval. */
void zcHandoffNotePollInterval(uint32_t commutation_interval);

/* Call from closed-loop path when a new interval is known. */
void zcHandoffNoteClosedInterval(uint32_t commutation_interval);

/* ZC_HANDOFF_ENTER_* — enter interrupt closed-loop when non-zero. */
uint8_t zcHandoffShouldEnterClosedLoop(uint32_t commutation_interval);

/* Non-zero => drop back to poll open-loop (old_routine = 1). */
uint8_t zcHandoffShouldExitClosedLoop(uint32_t average_interval);

#endif /* ZC_HANDOFF_H_ */
