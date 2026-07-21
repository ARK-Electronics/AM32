/*
 * zc_handoff.h - open-loop <-> closed-loop handoff
 *
 * Asymmetric hysteresis for the normal (non stall-protection) path:
 * closed-loop on real ZCs is the best signal; mode transitions are disruptive.
 * Exit only on positive evidence of failure.
 *
 * Enter: legacy CI < changeover; optional quality early-enter
 *        (ZC_HANDOFF_QUALITY_ENTER, default 0).
 *
 * Exit (every CL run — no enter-kind asymmetry):
 *   1. CI_ABS_MAX (average or instant CI)
 *   2. Hold if instant CI < changeover (lagging-average guard)
 *   3. Hold if average <= changeover+500
 *   4. Optional CV desync (ZC_HANDOFF_CV_EXIT, default 0) → sticky CL until
 *      near-stop so 50%→5% does not thrash OL↔CL
 *
 * Forced open-loop on stop/stall/reverse/signal-loss is outside this module.
 */
#ifndef ZC_HANDOFF_H_
#define ZC_HANDOFF_H_

#include <stdint.h>

#ifndef ZC_HANDOFF_CI_ABS_MAX
#	define ZC_HANDOFF_CI_ABS_MAX 12000u
#endif

#define ZC_HANDOFF_ENTER_NONE 0u
#define ZC_HANDOFF_ENTER_LEGACY 1u
#define ZC_HANDOFF_ENTER_QUALITY 2u

void zcHandoffReset(void);
void zcHandoffOnEnter(uint8_t enter_kind);
void zcHandoffOnExit(void);

void zcHandoffNotePollInterval(uint32_t commutation_interval);
void zcHandoffNoteClosedInterval(uint32_t commutation_interval);

uint8_t zcHandoffShouldEnterClosedLoop(uint32_t commutation_interval);
uint8_t zcHandoffShouldExitClosedLoop(uint32_t average_interval, uint32_t commutation_interval);

#endif /* ZC_HANDOFF_H_ */
