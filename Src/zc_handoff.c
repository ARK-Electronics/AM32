/*
 * zc_handoff.c - quality-based open-loop <-> closed-loop handoff
 */

#include "zc_handoff.h"
#include "motor_runtime.h"
#include "targets.h"

/* --- tunables (integer math only; no libm) --- */
#define ZC_HANDOFF_RING 8
#define ZC_HANDOFF_MIN_ZC 12
#define ZC_HANDOFF_MIN_SAMPLES 6
/* Enter when 100*sigma/mean < this (percent). */
#define ZC_HANDOFF_CV_ENTER_PCT 12u
/* Enter needs this many consecutive "good" poll intervals. */
#define ZC_HANDOFF_ENTER_STREAK 4u
/* Exit needs this many consecutive non-hold samples in the slow band. */
#define ZC_HANDOFF_EXIT_STREAK 6u

static uint16_t zc_ci_ring[ZC_HANDOFF_RING];
static uint8_t zc_ci_n;
static uint8_t zc_ci_i;
static uint8_t zc_enter_streak;
static uint8_t zc_exit_streak;

void zcHandoffReset(void)
{
	zc_ci_n = 0;
	zc_ci_i = 0;
	zc_enter_streak = 0;
	zc_exit_streak = 0;
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
	/* Keep the ring warm while closed-loop so slow-band exit CV is meaningful. */
	zc_ci_push(commutation_interval);
}

uint8_t zcHandoffShouldEnterClosedLoop(uint32_t commutation_interval)
{
	uint32_t cv;

	if (commutation_interval == 0u || commutation_interval > ZC_HANDOFF_CI_ABS_MAX) {
		zc_enter_streak = 0;
		return 0;
	}
	/*
	 * Fast path: legacy threshold — no zero_cross minimum (matches pre-handoff
	 * enter). Quality path below still requires MIN_ZC.
	 */
	if (commutation_interval < polling_mode_changeover) {
		return 1;
	}
	if (zero_crosses < ZC_HANDOFF_MIN_ZC) {
		zc_enter_streak = 0;
		return 0;
	}
	cv = zc_ci_cv_pct();
	if (cv <= ZC_HANDOFF_CV_ENTER_PCT && zc_ci_n >= ZC_HANDOFF_MIN_SAMPLES) {
		if (zc_enter_streak < 255u) {
			zc_enter_streak++;
		}
	} else {
		zc_enter_streak = 0;
	}
	return (uint8_t)(zc_enter_streak >= ZC_HANDOFF_ENTER_STREAK);
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
	 * Fast / hysteresis band (legacy guarantee): never quality-exit while
	 * average_interval <= polling_mode_changeover + 500. This matches the
	 * pre-handoff single-compare exit and avoids:
	 *  - confirm-reject thrash under load noise
	 *  - CV thrash during hard acceleration (slope looks like high sigma)
	 *  - COM-timer ISR cost of zc_ci_cv_pct() (divides + isqrt) at high eRPM
	 */
	if (average_interval <= polling_mode_changeover + 500u) {
		zc_exit_streak = 0;
		return 0;
	}

	/*
	 * Extended slow band (above legacy exit threshold): drop back unless
	 * interval quality is still excellent — low-KV hold after quality enter.
	 * High CV here means poll mode is safer.
	 */
	cv = zc_ci_cv_pct();
	if (zc_ci_n >= ZC_HANDOFF_MIN_SAMPLES && cv <= ZC_HANDOFF_CV_ENTER_PCT) {
		zc_exit_streak = 0;
		return 0;
	}

	if (zc_exit_streak < 255u) {
		zc_exit_streak++;
	}
	return (uint8_t)(zc_exit_streak >= ZC_HANDOFF_EXIT_STREAK);
}
