#pragma once

/*
 * Optionally rewrite the AM32 bootloader from an image embedded in the app.
 * No-op when EMBED_BOOTLOADER is not defined (e.g. HWCI_PERF builds).
 *
 * Call once early after clocks/flash are usable and before motors run.
 * May NVIC_SystemReset() after a successful update.
 */
void maybe_update_bootloader(void);
