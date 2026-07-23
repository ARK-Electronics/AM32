/*
 * faults.h - ESC fault and signal-loss handling
 *
 * Behavior-neutral extract of stuck-rotor, BEMF stall, and signal-timeout
 * policies previously inlined in setInput / tenKhzRoutine / main().
 */
#ifndef FAULTS_H_
#define FAULTS_H_

#include <stdint.h>

/* Optional fault IDs for future telemetry / logging (not yet exposed). */
typedef enum {
	FAULT_NONE = 0,
	FAULT_STUCK_ROTOR,
	FAULT_SIGNAL_TIMEOUT,
	FAULT_BEMF_STALL,
} fault_id_t;

/*
 * Stuck-rotor protection (was the top of setInput after throttle map).
 * If bemf_timeout_happened has exceeded the threshold, cut drive and latch.
 * Returns 1 when the fault is active (caller must skip normal input mapping).
 */
uint8_t faultHandleStuckRotorIfNeeded(void);

/*
 * 20 kHz signal-watchdog tick (end of tenKhzRoutine).
 * Implemented as RAM_FUNC so F051 RAM-resident tenKhzRoutine does not
 * pay a flash long-call veneer every tick.
 */
void faultSignalTimeoutTick(void);

/*
 * Main-loop poll: if signaltimeout is large enough, disarm and NVIC_SystemReset.
 * Armed: half second; disarmed: two seconds.
 */
void faultPollSignalTimeout(void);

/*
 * Main-loop BEMF timeout bookkeeping: clear counters under low throttle /
 * early run, and set bemf_timeout threshold from load.
 */
void faultUpdateBemfTimeoutPolicy(void);

/*
 * Main-loop stall: INTERVAL_TIMER_COUNT has run past the fixed stall window
 * while running. Increments bemf_timeout_happened and restarts ZC search.
 */
void faultHandleBemfIntervalStall(void);

/*
 * Episode-level desync rail (leaky bucket across restart cycles).
 *
 * Single desync episodes are already bounded (blind-step cap, miss bucket,
 * demag-late power cut). A bad EEPROM tune can still loop forever:
 * restart → spool → desync spike → restart. Each episode charges this
 * bucket; healthy closed-loop time drains it; zero throttle (pilot
 * intervention) clears it. At the limit, latch ESC_FAULT_STUCK so drive
 * stays off until the fault path clears.
 *
 * charge kinds: jump-check desync, stall-rail trip of an established run
 * (zero_crosses > 100; includes the blind/miss limit handoff, which always
 * follows an established closed loop), and future demag-late saturation.
 */
typedef enum {
	DESYNC_EPISODE_JUMP = 0,
	DESYNC_EPISODE_STALL_RAIL,
} desync_episode_kind_t;

void faultDesyncEpisodeCharge(desync_episode_kind_t kind);
/* 1 kHz: drain bucket when closed-loop is healthy; tick restart holdoff. */
void faultDesyncEpisodeTick1kHz(void);
/* True while a post-desync coast is mandatory (caller must not re-start). */
uint8_t faultDesyncRestartHoldoffActive(void);

#endif /* FAULTS_H_ */
