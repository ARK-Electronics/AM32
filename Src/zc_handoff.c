/*
 * zc_handoff.c - quality-based open-loop <-> closed-loop handoff
 */

#include "zc_handoff.h"
#include "motor_runtime.h"
#include "targets.h"

/* --- tunables (integer math only; no libm) --- */
#define ZC_HANDOFF_RING 8
/* Quality enter only (legacy fast path ignores this). Poll-mode CV can look
 * excellent almost immediately because open-loop is self-timed — need enough
 * ZCs that BEMF is real, not just a stable forced cadence. */
#define ZC_HANDOFF_MIN_ZC 40
#define ZC_HANDOFF_MIN_SAMPLES 6
/* Enter when 100*sigma/mean < this (percent). */
#define ZC_HANDOFF_CV_ENTER_PCT 12u
/* Hold in extended band when CV <= this (looser than enter → hysteresis). */
#define ZC_HANDOFF_CV_HOLD_PCT 18u
/* Enter needs this many consecutive "good" poll intervals. */
#define ZC_HANDOFF_ENTER_STREAK 8u
/* Exit needs this many consecutive non-hold samples in the slow band. */
#define ZC_HANDOFF_EXIT_STREAK 6u
/*
 * Quality enter only once CI is near the legacy band: changeover*2 + slack.
 * Open-loop can report low CV near stall without being interrupt-CL ready.
 */
#define ZC_HANDOFF_QUALITY_CI_SLACK 1000u

static uint16_t zc_ci_ring[ZC_HANDOFF_RING];
static uint8_t zc_ci_n;
static uint8_t zc_ci_i;
static uint8_t zc_enter_streak;
static uint8_t zc_exit_streak;
/* Non-zero only after a quality (not legacy) enter — enables extended-band hold.
 * OnEnter() calls Reset() then sets this; any other Reset() must clear it so a
 * direct caller cannot inherit a stale hold. */
static uint8_t zc_quality_hold;

void zcHandoffReset(void)
{
	zc_ci_n = 0;
	zc_ci_i = 0;
	zc_enter_streak = 0;
	zc_exit_streak = 0;
	zc_quality_hold = 0;
}

/*
 * Drop intervals that do not fit the ring (uint16). During a slow-down past
 * ~32.7 ms/commutation the ring freezes at its last "fast" contents; entry
 * is already gated by ZC_HANDOFF_CI_ABS_MAX before quality is consulted.
 */
static void zc_ci_push(uint32_t ci)
{
	if (ci == 0u || ci > 65535u) {
		return;
	}
	zc_ci_ring[zc_ci_i] = (uint16_t)ci;
	zc_ci_i++;
	if (zc_ci_i >= ZC_HANDOFF_RING) {
		zc_ci_i = 0;
	}
	if (zc_ci_n < ZC_HANDOFF_RING) {
		zc_ci_n++;
	}
}

static uint32_t zc_ci_mean(void)
{
	uint32_t sum = 0;
	uint8_t n = zc_ci_n;
	uint8_t k;
	if (n == 0) {
		return 0;
	}
	for (k = 0; k < n; k++) {
		sum += zc_ci_ring[k];
	}
	return sum / n;
}

/* Integer sqrt for 32-bit. */
static uint32_t isqrt32(uint32_t x)
{
	uint32_t r = 0;
	uint32_t b = 1u << 30;
	if (x < 2) {
		return x;
	}
	while (b > x) {
		b >>= 2;
	}
	while (b != 0) {
		if (x >= r + b) {
			x -= r + b;
			r = (r >> 1) + b;
		} else {
			r >>= 1;
		}
		b >>= 2;
	}
	return r;
}

/*
 * Return 100 * sigma / mean as percent, or 100 if not enough samples / mean 0.
 * Sum of squared diffs can exceed 2^32 (8 * 65535^2), so accumulate in
 * uint64_t; multiply via int64 to avoid signed-overflow UB on d*d.
 */
static uint32_t zc_ci_cv_pct(void)
{
	uint8_t n = zc_ci_n;
	uint8_t k;
	uint32_t mean;
	uint64_t acc = 0;
	if (n < ZC_HANDOFF_MIN_SAMPLES) {
		return 100;
	}
	mean = zc_ci_mean();
	if (mean < 1u) {
		return 100;
	}
	for (k = 0; k < n; k++) {
		int32_t d = (int32_t)zc_ci_ring[k] - (int32_t)mean;
		acc += (uint64_t)((int64_t)d * (int64_t)d);
	}
	/* sigma ~= sqrt(sum d^2 / n); var may exceed 2^32 — saturate for isqrt. */
	{
		uint64_t var = acc / n;
		uint32_t sigma = isqrt32(var > 0xffffffffull ? 0xffffffffu : (uint32_t)var);
		return (sigma * 100u) / mean;
	}
}

void zcHandoffNotePollInterval(uint32_t commutation_interval)
{
	if (zero_crosses < 3u) {
		zcHandoffReset();
		return;
	}
	zc_ci_push(commutation_interval);
}

RAM_FUNC void zcHandoffNoteClosedInterval(uint32_t commutation_interval)
{
	/* Keep the ring warm while closed-loop so quality-hold CV is meaningful. */
	zc_ci_push(commutation_interval);
}

uint8_t zcHandoffShouldEnterClosedLoop(uint32_t commutation_interval)
{
	uint32_t cv;
	uint32_t quality_ci_max;

	if (commutation_interval == 0u || commutation_interval > ZC_HANDOFF_CI_ABS_MAX) {
		zc_enter_streak = 0;
		return ZC_HANDOFF_ENTER_NONE;
	}
	/*
	 * Fast path: legacy threshold — no zero_cross minimum (matches pre-handoff
	 * enter). Caller must treat this as non-quality (no extended-band hold).
	 */
	if (commutation_interval < polling_mode_changeover) {
		return ZC_HANDOFF_ENTER_LEGACY;
	}
	/* Not yet near the legacy band: poll CV is not a CL-readiness signal. */
	quality_ci_max = polling_mode_changeover * 2u + ZC_HANDOFF_QUALITY_CI_SLACK;
	if (commutation_interval > quality_ci_max) {
		zc_enter_streak = 0;
		return ZC_HANDOFF_ENTER_NONE;
	}
	if (zero_crosses < ZC_HANDOFF_MIN_ZC) {
		zc_enter_streak = 0;
		return ZC_HANDOFF_ENTER_NONE;
	}
	cv = zc_ci_cv_pct();
	if (cv <= ZC_HANDOFF_CV_ENTER_PCT && zc_ci_n >= ZC_HANDOFF_MIN_SAMPLES) {
		if (zc_enter_streak < 255u) {
			zc_enter_streak++;
		}
	} else {
		zc_enter_streak = 0;
	}
	if (zc_enter_streak >= ZC_HANDOFF_ENTER_STREAK) {
		return ZC_HANDOFF_ENTER_QUALITY;
	}
	return ZC_HANDOFF_ENTER_NONE;
}

RAM_FUNC uint8_t zcHandoffShouldExitClosedLoop(uint32_t average_interval)
{
	uint32_t cv;

	/* Near-stop / lost BEMF: always drop to poll. */
	if (average_interval > ZC_HANDOFF_CI_ABS_MAX) {
		zc_exit_streak = ZC_HANDOFF_EXIT_STREAK;
		return 1;
	}

	/*
	 * Fast / hysteresis band (legacy guarantee): never exit while
	 * average_interval <= polling_mode_changeover + 500.
	 */
	if (average_interval <= polling_mode_changeover + 500u) {
		zc_exit_streak = 0;
		return 0;
	}

	/*
	 * Extended band (average still above legacy exit threshold).
	 *
	 * After *legacy* enter, average lags CI during spool-up — so we can be
	 * here with a healthy interrupt loop. Pre-handoff always exited on this
	 * compare alone; do the same unless this run was a quality enter that is
	 * allowed to hold on good CV (low-KV early CL).
	 */
	if (!zc_quality_hold) {
		return 1;
	}

	/* Quality hold: underfilled ring after enter-reset is not "bad". */
	if (zc_ci_n < ZC_HANDOFF_MIN_SAMPLES) {
		zc_exit_streak = 0;
		return 0;
	}
	cv = zc_ci_cv_pct();
	if (cv <= ZC_HANDOFF_CV_HOLD_PCT) {
		zc_exit_streak = 0;
		return 0;
	}

	if (zc_exit_streak < 255u) {
		zc_exit_streak++;
	}
	return (uint8_t)(zc_exit_streak >= ZC_HANDOFF_EXIT_STREAK);
}

void zcHandoffOnEnter(uint8_t enter_kind)
{
	zcHandoffReset();
	/* Arm after Reset so quality hold is never left set by a bare Reset(). */
	zc_quality_hold = (enter_kind == ZC_HANDOFF_ENTER_QUALITY) ? 1u : 0u;
}

void zcHandoffOnExit(void)
{
	zcHandoffReset();
}
