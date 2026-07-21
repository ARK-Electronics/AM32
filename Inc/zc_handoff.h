/*
 * zc_handoff.h - open-loop <-> closed-loop handoff
 *
 * Enter (normal path, not stall-protection):
 *   - Legacy: CI < polling_mode_changeover (same as pre-handoff).
 *   - Optional quality early-enter (ZC_HANDOFF_QUALITY_ENTER=1): near-legacy
 *     CI + stable poll CV after enough ZCs — for low-KV crawl; off by default
 *     because sticky CL + premature enter can trap a desynced loop.
 *
 * Exit: only CI_ABS_MAX (near stop / lost BEMF). Prefer staying in closed-loop
 * once entered — no OL↔CL thrash on changeover+500 or CV. Forced open-loop on
 * stop / stall / reverse / signal loss is outside this module.
 *
 * Call zcHandoffOnEnter() after enter, zcHandoffOnExit() on drop to poll or
 * forced open-loop.
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
void zcHandoffOnEnter(uint8_t enter_kind);
void zcHandoffOnExit(void);

void zcHandoffNotePollInterval(uint32_t commutation_interval);
void zcHandoffNoteClosedInterval(uint32_t commutation_interval);

uint8_t zcHandoffShouldEnterClosedLoop(uint32_t commutation_interval);
uint8_t zcHandoffShouldExitClosedLoop(uint32_t average_interval);

#endif /* ZC_HANDOFF_H_ */
