/*
 * faults.c - extracted from main/control_loop (behavior-neutral)
 */

#include "faults.h"

#include "main.h"
#include "common.h"
#include "motor_runtime.h"
#include "phaseouts.h"
#include "comparator.h"
#include "peripherals.h"
#include "functions.h"
#include "eeprom.h"
#include "signal.h"
#include "commutation.h"
#include "targets.h"
#include "esc_state.h"
#include "IO.h"
#include "sounds.h"

#ifdef USE_RGB_LED
extern void setIndividualRGBLed(uint8_t, uint8_t, uint8_t);
#endif

extern volatile uint16_t zero_input_count;
extern volatile uint32_t dma_buffer[64];
extern void resetInputCaptureTimer(void);

uint8_t faultHandleStuckRotorIfNeeded(void)
{
#ifndef BRUSHED_MODE
	if ((bemf_timeout_happened > bemf_timeout) && eepromBuffer.stuck_rotor_protection) {
		allOff();
		maskPhaseInterrupts();
		escToFaultStuck();
#	ifdef USE_RGB_LED
		setIndividualRGBLed(1, 0, 0);
#	endif
		return 1;
	}
#endif
	return 0;
}

/* RAM-resident: called from tenKhzRoutine every 50 us on F051. */
RAM_FUNC void faultSignalTimeoutTick(void)
{
#if defined(FIXED_DUTY_MODE) || defined(FIXED_SPEED_MODE)
	if (getInputPinState()) {
		signaltimeout++;
		if (signaltimeout > LOOP_FREQUENCY_HZ) {
			bootSoundMarkSignalLost();
			NVIC_SystemReset();
		}
	} else {
		signaltimeout = 0;
	}
#else
	signaltimeout++;
#endif
}

void faultPollSignalTimeout(void)
{
	if (signaltimeout > (LOOP_FREQUENCY_HZ >> 1)) { // half second timeout when armed;
		if (escIsArmed()) {
			allOff();
			escToFaultSignal();
			zero_input_count = 0;
			SET_DUTY_CYCLE_ALL(0);
			resetInputCaptureTimer();
			for (int i = 0; i < 64; i++) {
				dma_buffer[i] = 0;
			}
			bootSoundMarkSignalLost();
			NVIC_SystemReset();
		}
		if (signaltimeout > LOOP_FREQUENCY_HZ << 1) { // 2 second when not armed
			allOff();
			escToFaultSignal();
			zero_input_count = 0;
			SET_DUTY_CYCLE_ALL(0);
			resetInputCaptureTimer();
			for (int i = 0; i < 64; i++) {
				dma_buffer[i] = 0;
			}
			bootSoundMarkSignalLost();
			NVIC_SystemReset();
		}
	}
}

void faultUpdateBemfTimeoutPolicy(void)
{
#ifndef BRUSHED_MODE
	if ((zero_crosses > 1000) || (adjusted_input == 0)) {
		bemf_timeout_happened = 0;
	}
	if (adjusted_input == 0) {
		// Zero throttle is pilot intervention: clear the episode rail the
		// same way the legacy latch clears. Deliberately NOT on
		// zero_crosses > 1000 - a bad-tune cycle that respins fast between
		// desyncs must keep accumulating (that reset defeating the stuck
		// latch is one of the gaps this rail exists to close).
		desync_episode_bucket = 0;
		desync_restart_holdoff_ms = 0;
	}
	if (zero_crosses > 100 && adjusted_input < 200) {
		bemf_timeout_happened = 0;
	}
	if (eepromBuffer.use_sine_start && adjusted_input < 160) {
		bemf_timeout_happened = 0;
	}

	if (crawler_mode) {
		if (adjusted_input < 400) {
			bemf_timeout_happened = 0;
		}
	} else {
		if (adjusted_input < 150) { // startup duty cycle should be low enough to not burn motor
			bemf_timeout = 100;
		} else {
			bemf_timeout = 10;
		}
	}
#endif
}

/*
 * Cross-episode desync protection. Per-episode rails (blind-step cap, miss
 * bucket, demag-late power cut) bound a single event; this bucket bounds a
 * *repeating* restart→desync cycle from a bad tune / wrong prop match.
 *
 * Charge rates are tuned so 3-5 hard episodes latch within ~1-2 s of cycling,
 * while a couple of honest in-flight desyncs drain out in a few seconds of
 * healthy closed-loop (stall: 4 episodes at 12; jump: 5 at 8).
 *
 * The FIRST episode must restart immediately - an honest in-flight desync
 * that coasts even 100 ms is a dropped motor on a quad - so restart holdoff
 * only arms once the bucket shows repetition (>= MIN_BUCKET, i.e. from the
 * second episode on) and then grows so the FETs spend most of each later
 * cycle cooling instead of immediately re-entering the wrong-phase spike.
 * Stall path: ep2 300 ms, ep3 500 ms, ep4 latch. Jump path: ep2 100 ms,
 * ep3 300 ms, ep4 500 ms, ep5 latch.
 */
#define DESYNC_EPISODE_LIMIT 40
#define DESYNC_EPISODE_CHARGE_JUMP 8
#define DESYNC_EPISODE_CHARGE_STALL 12
#define DESYNC_EPISODE_DRAIN_MS 100  /* -1 charge per this many ms of healthy CL */
#define DESYNC_BACKOFF_MIN_BUCKET 16 /* below this (first episode): no holdoff */
#define DESYNC_BACKOFF_BASE_MS 100
#define DESYNC_BACKOFF_STEP_MS 25
#define DESYNC_BACKOFF_MAX_MS 500

/*
 * Blind-GRIND rail. The episode rail above only sees DISCRETE failures
 * (jump desync, stall trip). Bench 2026-07-23 (pr48-snap-rail-1, 900KV +
 * 10x5x3 6S, snap 0.20->0.55): a spinning snap can drop the loop into a
 * CONTINUOUS partial desync that sits below every threshold - 423 blind
 * steps/s at a 19.5% miss rate (miss bucket needs >25% and net-drains;
 * consecutive counter resets on every accepted crossing; CI step +45% is
 * under the jump check's 50%; the stall rail never fires because blind
 * steps keep resetting INTERVAL_TIMER) - while the power cut engages and
 * releases in a limit cycle (one accepted crossing resets zc_blind_steps,
 * duty re-slews at max_ramp, spike to 102 A, desync, repeat).
 *
 * The unambiguous signal is the blind-step RATE: healthy runs total 9-27
 * blind steps per RUN; the grind produces 40+ per 100 ms. Sample
 * zc_blind_window_count each 100 ms; ONE hot window holds the power cut
 * AND forces the stall rail (mask + kick INTERVAL_TIMER, same primitives
 * as the blind-limit handoff), which restarts the loop and charges the
 * episode bucket (zero_crosses is far above 100 in a grind), so
 * repetition escalates through backoff to the latch like any other
 * episode source.
 *
 * Restart on the FIRST hot window, not after N consecutive: requiring
 * consecutive hot windows is self-defeating (bench pr48-snap-rail-2) -
 * the held cut drops the blind rate below threshold, the streak resets,
 * no restart ever fires, and each hold expiry re-slews duty into a fresh
 * spike (33-88 A at ~600 ms period). The detection margin carries a
 * single-window trigger, and a false trip costs one restart plus one
 * episode charge that drains in ~1.2 s of healthy running.
 */
#define GRIND_WINDOW_MS 100
#define GRIND_WINDOW_BLIND_LIMIT 15 /* >=150 blind/s = grind; healthy bursts stay single-digit */
#define GRIND_HOLD_MS 250

static uint8_t desync_episode_drain_ms;
static uint8_t grind_window_ms;

static void desync_episode_apply_backoff(void)
{
	if (desync_episode_bucket < DESYNC_BACKOFF_MIN_BUCKET) {
		return; // first episode restarts immediately (legacy behavior)
	}
	uint16_t hold =
		(uint16_t)(DESYNC_BACKOFF_BASE_MS + (uint16_t)(desync_episode_bucket - DESYNC_BACKOFF_MIN_BUCKET) * DESYNC_BACKOFF_STEP_MS);
	if (hold > DESYNC_BACKOFF_MAX_MS) {
		hold = DESYNC_BACKOFF_MAX_MS;
	}
	if (hold > desync_restart_holdoff_ms) {
		desync_restart_holdoff_ms = hold;
	}
}

void faultDesyncEpisodeCharge(desync_episode_kind_t kind)
{
#ifndef BRUSHED_MODE
	uint8_t inc = DESYNC_EPISODE_CHARGE_STALL;
	if (kind == DESYNC_EPISODE_JUMP) {
		inc = DESYNC_EPISODE_CHARGE_JUMP;
	}
	if ((uint16_t)desync_episode_bucket + inc > 255) {
		desync_episode_bucket = 255;
	} else {
		desync_episode_bucket = (uint8_t)(desync_episode_bucket + inc);
	}
	desync_episode_drain_ms = 0;
	desync_episode_apply_backoff();

	// Learned ramp back-off: a desync episode means the configured ramp
	// outran what this motor/prop can follow, and no reactive signal can
	// catch it in time (the first ~15 ms of a too-fast transient are
	// electrically silent - see the note at the slew limiter in
	// control_loop.c). Clamp all ramp regimes to the fine rate (0.1%/ms,
	// the rate bench-proven to hold lock on the heaviest supported prop)
	// for the REST OF THE POWER CYCLE: the first episode is the lesson,
	// every later transition stays locked, and the episode bucket stops
	// charging so the latch never fires from ramp misconfiguration alone.
	// Deliberately not cleared at zero throttle (unlike the bucket):
	// re-arming the fast ramp would re-desync on the next transient.
	// A settings write reruns loadEEpromSettings and restores the
	// configured values - retuning is the way back within a session.
	ramp_divider = 9;
	max_ramp_startup = 1;
	max_ramp_low_rpm = 1;
	max_ramp_high_rpm = 1;

	if (desync_episode_bucket >= DESYNC_EPISODE_LIMIT) {
		allOff();
		maskPhaseInterrupts();
		escToFaultStuck();
#	ifdef USE_RGB_LED
		setIndividualRGBLed(1, 0, 0);
#	endif
	}
#else
	(void)kind;
#endif
}

void faultDesyncEpisodeTick1kHz(void)
{
#ifndef BRUSHED_MODE
	if (desync_restart_holdoff_ms > 0) {
		desync_restart_holdoff_ms--;
	}
	if (zc_grind_hold_ms > 0) {
		zc_grind_hold_ms--;
	}

	/* Blind-grind window (see block comment above the constants). */
	if (++grind_window_ms >= GRIND_WINDOW_MS) {
		grind_window_ms = 0;
		uint8_t blind_in_window = zc_blind_window_count;
		zc_blind_window_count = 0; // a step between read and clear is one lost count - fine
		if (blind_in_window >= GRIND_WINDOW_BLIND_LIMIT && running && !old_routine) {
			// Hot window: hold the power cut (bridges the gap until the
			// restart takes and covers any deferred path) and force the
			// stall rail, same primitives as the blind-limit handoff in
			// PeriodElapsedCallback: with phase interrupts masked and
			// the deadline disarmed, nothing can reset INTERVAL_TIMER
			// before the main loop runs faultHandleBemfIntervalStall,
			// which restarts and charges the episode bucket.
			zc_grind_hold_ms = GRIND_HOLD_MS;
			maskPhaseInterrupts();
			DISABLE_COM_TIMER_INT();
			zc_deadline_armed = 0;
			SET_INTERVAL_TIMER_COUNT(46000);
		}
	}

	/* Drain only in established, trusted closed loop. bemf_timeout_happened
	 * is deliberately NOT in the gate: it stays nonzero until
	 * zero_crosses > 1000 (5-10 s at crawler rpm), which would block drain
	 * through exactly the healthy running that should earn it. */
	if (escInClosedLoop() && zc_blind_steps == 0 && zc_demag_run == 0 && desync_episode_bucket > 0) {
		if (desync_episode_drain_ms < 255) {
			desync_episode_drain_ms++;
		}
		if (desync_episode_drain_ms >= DESYNC_EPISODE_DRAIN_MS) {
			desync_episode_drain_ms = 0;
			desync_episode_bucket--;
		}
	} else {
		desync_episode_drain_ms = 0;
	}
#endif
}

uint8_t faultDesyncRestartHoldoffActive(void)
{
	return (uint8_t)(desync_restart_holdoff_ms > 0);
}

void faultHandleBemfIntervalStall(void)
{
	/* Six-step only (not sine soft-start). */
	if (INTERVAL_TIMER_COUNT > 45000 && (escInOpenLoop() || escInClosedLoop())) {
		bemf_timeout_happened++;

		maskPhaseInterrupts();
		// Charge the episode rail only when this run was ESTABLISHED
		// before it died (the bad-tune restart->spool->desync cycle
		// always reaches closed loop first). A start attempt that never
		// got going is the legacy stuck-rotor rail's job, with its
		// throttle-scaled tolerance (bemf_timeout 100 below input 150) -
		// heavy props legitimately kick many times at low throttle, and
		// charging those latched the ESC on the 4th kick. This gate also
		// covers the blind/miss-limit handoff (bemf_zc kicks
		// INTERVAL_TIMER to 46000 with comparator interrupts masked, so
		// this rail is guaranteed to run next pass): blind stepping only
		// arms at zero_crosses >= 100, so those episodes always charge.
		if (zero_crosses > 100) {
			faultDesyncEpisodeCharge(DESYNC_EPISODE_STALL_RAIL);
		}
		if (escIsFault()) {
			/* Episode rail latched: do not re-enter startup. */
			zero_crosses = 0;
			running = 0;
			return;
		}
		escNoteStallOrDesync(1);
		zero_crosses = 0;
		if (faultDesyncRestartHoldoffActive()) {
			/* Coast until holdoff expires; main loop will re-arm. */
			running = 0;
			allOff();
			return;
		}
		zcfoundroutine();
	}
}
