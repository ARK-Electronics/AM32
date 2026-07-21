/*
 * zc_handoff.c - quality-based open-loop <-> closed-loop handoff
 */

#include "zc_handoff.h"
#include "motor_runtime.h"

/* --- tunables (integer math only; no libm) --- */
#define ZC_HANDOFF_RING 8
#define ZC_HANDOFF_MIN_ZC 12
#define ZC_HANDOFF_MIN_SAMPLES 6
/* Enter when 100*sigma/mean < this (percent). */
#define ZC_HANDOFF_CV_ENTER_PCT 12u
/* Exit when 100*sigma/mean > this (percent). Hysteresis vs enter. */
#define ZC_HANDOFF_CV_EXIT_PCT 28u
/* Enter needs this many consecutive "good" poll intervals. */
#define ZC_HANDOFF_ENTER_STREAK 4u
/* Exit needs this many consecutive bad closed-loop samples. */
#define ZC_HANDOFF_EXIT_STREAK 6u
/* ISR confirm: exit if rejects exceed this fraction of (accept+reject) window. */
#define ZC_HANDOFF_REJECT_WINDOW 16u
#define ZC_HANDOFF_REJECT_EXIT_NUM 8u /* 8/16 = 50% */

static uint16_t zc_ci_ring[ZC_HANDOFF_RING];
static uint8_t zc_ci_n;
static uint8_t zc_ci_i;
static uint8_t zc_enter_streak;
static uint8_t zc_exit_streak;

/* Saturating 0..window style counters for recent confirms. */
static uint8_t zc_confirm_accepts;
static uint8_t zc_confirm_rejects;

void zcHandoffReset(void)
{
	zc_ci_n = 0;
	zc_ci_i = 0;
	zc_enter_streak = 0;
	zc_exit_streak = 0;
	zc_confirm_accepts = 0;
	zc_confirm_rejects = 0;
}

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

/*
 * Return 100 * sigma / mean as percent, or 100 if not enough samples / mean 0.
 * Uses: 10000 * sum(d^2) < thr^2 * mean^2 * n  for comparisons elsewhere;
 * here returns approximate cv% via integer sqrt of (var).
 */
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

/* Integer sqrt for 32-bit (enough for sum of squared 16-bit diffs). */
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

static uint32_t zc_ci_cv_pct(void)
{
	uint8_t n = zc_ci_n;
	uint8_t k;
	uint32_t mean;
	uint32_t acc = 0;
	if (n < ZC_HANDOFF_MIN_SAMPLES) {
		return 100;
	}
	mean = zc_ci_mean();
	if (mean < 1u) {
		return 100;
	}
	for (k = 0; k < n; k++) {
		int32_t d = (int32_t)zc_ci_ring[k] - (int32_t)mean;
		acc += (uint32_t)(d * d);
	}
	/* sigma ~= sqrt(sum d^2 / n) */
	{
		uint32_t var = acc / n;
		uint32_t sigma = isqrt32(var);
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

void zcHandoffNoteClosedInterval(uint32_t commutation_interval)
{
	/* Keep the ring warm while closed-loop so exit CV is meaningful. */
	zc_ci_push(commutation_interval);
}

void zcHandoffNoteConfirmReject(void)
{
	if (zc_confirm_rejects < 255u) {
		zc_confirm_rejects++;
	}
	/* Decay accepts so the window tracks recent behavior. */
	if (zc_confirm_accepts > 0u) {
		zc_confirm_accepts--;
	}
	if ((uint16_t)zc_confirm_accepts + (uint16_t)zc_confirm_rejects > ZC_HANDOFF_REJECT_WINDOW) {
		if (zc_confirm_accepts > zc_confirm_rejects) {
			zc_confirm_accepts = (uint8_t)(ZC_HANDOFF_REJECT_WINDOW - zc_confirm_rejects);
		} else if (zc_confirm_rejects > 0u) {
			zc_confirm_rejects--;
		}
	}
}

void zcHandoffNoteConfirmAccept(void)
{
	if (zc_confirm_accepts < 255u) {
		zc_confirm_accepts++;
	}
	if (zc_confirm_rejects > 0u) {
		zc_confirm_rejects--;
	}
	if ((uint16_t)zc_confirm_accepts + (uint16_t)zc_confirm_rejects > ZC_HANDOFF_REJECT_WINDOW) {
		if (zc_confirm_rejects > 0u) {
			zc_confirm_rejects--;
		} else if (zc_confirm_accepts > ZC_HANDOFF_REJECT_WINDOW) {
			zc_confirm_accepts = ZC_HANDOFF_REJECT_WINDOW;
		}
	}
}

uint8_t zcHandoffShouldEnterClosedLoop(uint32_t commutation_interval)
{
	uint32_t cv;
	if (zero_crosses < ZC_HANDOFF_MIN_ZC) {
		zc_enter_streak = 0;
		return 0;
	}
	if (commutation_interval == 0u || commutation_interval > ZC_HANDOFF_CI_ABS_MAX) {
		zc_enter_streak = 0;
		return 0;
	}
	/* Fast path: already electrically quick (legacy threshold). */
	if (commutation_interval < polling_mode_changeover) {
		zc_enter_streak = ZC_HANDOFF_ENTER_STREAK;
		return 1;
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

uint8_t zcHandoffShouldExitClosedLoop(uint32_t average_interval)
{
	uint32_t cv;
	uint8_t bad = 0;
	uint16_t conf_total;

	/* Near-stop / lost BEMF: always drop to poll. */
	if (average_interval > ZC_HANDOFF_CI_ABS_MAX) {
		zc_exit_streak = ZC_HANDOFF_EXIT_STREAK;
		return 1;
	}

	cv = zc_ci_cv_pct();
	if (zc_ci_n >= ZC_HANDOFF_MIN_SAMPLES && cv > ZC_HANDOFF_CV_EXIT_PCT) {
		bad = 1;
	}

	conf_total = (uint16_t)zc_confirm_accepts + (uint16_t)zc_confirm_rejects;
	if (conf_total >= (ZC_HANDOFF_REJECT_WINDOW / 2u) && zc_confirm_rejects >= ZC_HANDOFF_REJECT_EXIT_NUM) {
		bad = 1;
	}

	if (bad) {
		if (zc_exit_streak < 255u) {
			zc_exit_streak++;
		}
	} else {
		zc_exit_streak = 0;
	}
	return (uint8_t)(zc_exit_streak >= ZC_HANDOFF_EXIT_STREAK);
}
