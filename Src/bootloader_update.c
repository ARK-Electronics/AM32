/*
 * App-side bootloader update (ArduPilot-style).
 *
 * When EMBED_BOOTLOADER is set, compare flash @ MCU base to the embedded
 * image; if different, program the BL region with IRQs off, verify, reset.
 *
 * HWCI_PERF builds omit EMBED_BOOTLOADER so the ~4 KiB image is not linked.
 */

#include "bootloader_update.h"

#include "eeprom.h"
#include "main.h"
#include "targets.h"

#include <stdint.h>
#include <string.h>

#if defined(EMBED_BOOTLOADER) && defined(MCU_F051)

/* ARK 4IN1 / HARDWARE_GROUP_F0_B signal pin is PB4. */
#	include "bootloader_images/bl_image_f051_pb4.h"

#	ifndef MCU_FLASH_START
#		define MCU_FLASH_START 0x08000000u
#	endif

/* F051 page = 1 KiB; erase runs when address is page-aligned in save_flash_nolib. */
#	define BL_CHUNK_BYTES 256u

void maybe_update_bootloader(void)
{
	const uint32_t len = (uint32_t)sizeof(bl_image);
	const uint8_t *want = bl_image;
	const uint8_t *have = (const uint8_t *)(uintptr_t)MCU_FLASH_START;

	if (len == 0u || (len & 1u) != 0u) {
		return;
	}

	if (memcmp(have, want, len) == 0) {
		return;
	}

	/*
	 * Same flash bank as this code on F051, but we only erase/program the
	 * bootloader pages below the app. Disable IRQs so no vector fetch races
	 * a half-written state mid-chunk (mirrors AM32-bootloader bl_update).
	 */
	__disable_irq();

	uint32_t off = 0;
	while (off < len) {
		uint32_t chunk = BL_CHUNK_BYTES;
		if (chunk > (len - off)) {
			chunk = len - off;
		}

		/* Non-const pointer: save_flash_nolib only reads the buffer. */
		uint8_t *src = (uint8_t *)(uintptr_t)&want[off];
		const uint32_t addr = MCU_FLASH_START + off;

		save_flash_nolib(src, (int)chunk, addr);

		if (memcmp((const void *)(uintptr_t)addr, &want[off], chunk) != 0) {
			/* Retry this chunk until it sticks (power loss = brick either way). */
			continue;
		}
		off += chunk;
	}

	/* Full-image check before reset. */
	if (memcmp(have, want, len) != 0) {
		/* Last-ditch: restart from offset 0. */
		off = 0;
		while (off < len) {
			uint32_t chunk = BL_CHUNK_BYTES;
			if (chunk > (len - off)) {
				chunk = len - off;
			}
			uint8_t *src = (uint8_t *)(uintptr_t)&want[off];
			const uint32_t addr = MCU_FLASH_START + off;
			save_flash_nolib(src, (int)chunk, addr);
			if (memcmp((const void *)(uintptr_t)addr, &want[off], chunk) != 0) {
				continue;
			}
			off += chunk;
		}
	}

	NVIC_SystemReset();
}

#else /* !EMBED_BOOTLOADER || !MCU_F051 */

void maybe_update_bootloader(void)
{
	/* Not embedded (HWCI) or MCU not supported in this prototype. */
}

#endif
