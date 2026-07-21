/*
 * zc_handoff.c - open-loop <-> closed-loop handoff
 *
 * Asymmetric hysteresis: once closed-loop is commutating on real ZCs, stay
 * there. The mode transition is the disruptive event. Exit only on positive
 * evidence of failure (near-stop; optional CV desync). Forced open-loop on
 * stop/stall/reverse/signal-loss is outside this module.
 */

#include "zc_handoff.h"
#include "motor_runtime.h"
#include "targets.h"

#define ZC_HANDOFF_RING 8
#define ZC_HANDOFF_MIN_ZC 40
#define ZC_HANDOFF_MIN_SAMPLES 6
#define ZC_HANDOFF_CV_ENTER_PCT 12u
#define ZC_HANDOFF_CV_EXIT_PCT 40u
#define ZC_HANDOFF_ENTER_STREAK 8u
#define ZC_HANDOFF_EXIT_STREAK 10u
#define ZC_HANDOFF_QUALITY_CI_SLACK 1000u

/* Quality early-enter: off until thrust-stand validation with sticky CL. */
#ifndef ZC_HANDOFF_QUALITY_ENTER
#	define ZC_HANDOFF_QUALITY_ENTER 0
#endif

/*
 * CV exit in the extended band. Off by default — stay in CL until CI_ABS_MAX
 * (preferred for sound: no thrash on 50%→5%). Set 1 to enable loose CV desync
 * drop once lagging-CI guard is proven on the stand.
 */
#ifndef ZC_HANDOFF_CV_EXIT
#	define ZC_HANDOFF_CV_EXIT 0
#endif

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

#if ZC_HANDOFF_QUALITY_ENTER || ZC_HANDOFF_CV_EXIT
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
	{
		uint64_t var = acc / n;
		uint32_t sigma = isqrt32(var > 0xffffffffull ? 0xffffffffu : (uint32_t)var);
		return (sigma * 100u) / mean;
	}
}
#endif

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
	zc_ci_push(commutation_interval);
}

uint8_t zcHandoffShouldEnterClosedLoop(uint32_t commutation_interval)
{
	if (commutation_interval == 0u || commutation_interval > ZC_HANDOFF_CI_ABS_MAX) {
		zc_enter_streak = 0;
		return ZC_HANDOFF_ENTER_NONE;
	}
	/* Legacy enter (pre-handoff). */
	if (commutation_interval < polling_mode_changeover) {
		return ZC_HANDOFF_ENTER_LEGACY;
	}

#if ZC_HANDOFF_QUALITY_ENTER
	{
		uint32_t cv;
		uint32_t quality_ci_max = polling_mode_changeover * 2u + ZC_HANDOFF_QUALITY_CI_SLACK;
		if (commutation_interval > quality_ci_max || zero_crosses < ZC_HANDOFF_MIN_ZC) {
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
	}
#else
	zc_enter_streak = 0;
#endif
	return ZC_HANDOFF_ENTER_NONE;
}

/*
 * Exit for every closed-loop run (no quality_hold asymmetry).
 * 1) CI_ABS_MAX  2) lagging-CI guard  3) average fast band  4) optional CV
 */
RAM_FUNC uint8_t zcHandoffShouldExitClosedLoop(uint32_t average_interval, uint32_t commutation_interval)
{
	if (average_interval > ZC_HANDOFF_CI_ABS_MAX || commutation_interval > ZC_HANDOFF_CI_ABS_MAX) {
		zc_exit_streak = ZC_HANDOFF_EXIT_STREAK;
		return 1;
	}

	/* Instant CI already fast → avg is stale (spool-up). Hold, no CV. */
	if (commutation_interval < polling_mode_changeover) {
		zc_exit_streak = 0;
		return 0;
	}

	if (average_interval <= polling_mode_changeover + 500u) {
		zc_exit_streak = 0;
		return 0;
	}

#if ZC_HANDOFF_CV_EXIT
	if (zc_ci_n < ZC_HANDOFF_MIN_SAMPLES) {
		zc_exit_streak = 0;
		return 0;
	}
	{
		uint32_t cv = zc_ci_cv_pct();
		if (cv <= ZC_HANDOFF_CV_EXIT_PCT) {
			zc_exit_streak = 0;
			return 0;
		}
	}
	if (zc_exit_streak < 255u) {
		zc_exit_streak++;
	}
	return (uint8_t)(zc_exit_streak >= ZC_HANDOFF_EXIT_STREAK);
#else
	/* Sticky CL through crawl until near-stop (50%→5% stays closed-loop). */
	zc_exit_streak = 0;
	return 0;
#endif
}

void zcHandoffOnEnter(uint8_t enter_kind)
{
	(void)enter_kind;
	zcHandoffReset();
}

void zcHandoffOnExit(void)
{
	zcHandoffReset();
}
