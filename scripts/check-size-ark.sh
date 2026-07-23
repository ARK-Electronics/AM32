#!/usr/bin/env bash
# Fail if the ARK F051 (+ HWCI_PERF) image exceeds flash/RAM budgets.
#
# STM32F051K6 application region (from linker script / build map):
#   FLASH (app): 27424 bytes
#   RAM:         8000 bytes
#
# We require a small free margin so a "tiny" PR cannot silently fill the chip.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

# STM32F051K6TX_FLASH.ld regions that hold RX image (not EEPROM):
#   FLASH_VECTAB 192 + FLASH 27424 + FILE_NAME 32 = 27648
# arm-none-eabi-size "text" spans those RX regions; do not compare to 27424 alone.
FLASH_CAPACITY="${FLASH_CAPACITY:-27648}"
RAM_CAPACITY="${RAM_CAPACITY:-8000}"
# After -Os (+ RAM_FUNC -O3), const pwmSin, no-heap, inlined esc predicates,
# HWCI sits ~88% flash / ~80% RAM. Leave margin for accidental growth.
# (Global -O3 filled ~99% and regressed hold100 free-run on F051 — do not raise
# this limit just to fit -O3.)
FLASH_MAX_PCT="${FLASH_MAX_PCT:-95}"
RAM_MAX_PCT="${RAM_MAX_PCT:-90}"

ELF=$(ls -1 obj/AM32_ARK_4IN1_F051_*.elf 2>/dev/null | head -1 || true)
if [ -z "$ELF" ] || [ ! -f "$ELF" ]; then
  echo "error: no obj/AM32_ARK_4IN1_F051_*.elf — build ARK_4IN1_F051 HWCI_PERF=1 first" >&2
  exit 2
fi

SIZE_BIN="${SIZE_BIN:-}"
if [ -z "$SIZE_BIN" ]; then
  if command -v arm-none-eabi-size >/dev/null 2>&1; then
    SIZE_BIN=arm-none-eabi-size
  else
    # Prefer the repo xPack toolchain if present
    for c in \
      tools/linux/xpack-arm-none-eabi-gcc-15.2.1-1.1/bin/arm-none-eabi-size \
      tools/linux/xpack-arm-none-eabi-gcc-14.2.1-1.1/bin/arm-none-eabi-size \
      tools/linux/xpack-arm-none-eabi-gcc-10.3.1-2.3/bin/arm-none-eabi-size
    do
      if [ -x "$ROOT/$c" ]; then SIZE_BIN="$ROOT/$c"; break; fi
    done
  fi
fi
if [ -z "$SIZE_BIN" ]; then
  echo "error: arm-none-eabi-size not found" >&2
  exit 127
fi

# Section-accurate accounting (matches ld --print-memory-usage better than
# the classic one-line size totals on this multi-region F051 layout).
SEC=$("$SIZE_BIN" -A "$ELF")
sec_size() { echo "$SEC" | awk -v s="$1" '$1==s{print $2; found=1} END{if(!found)print 0}'; }

TEXT=$(sec_size .text)
RODATA=$(sec_size .rodata)
ISR=$(sec_size .isr_vector)
FILE_NAME_SZ=$(sec_size .file_name)
INITA=$(sec_size .init_array)
FINIA=$(sec_size .fini_array)
DATA=$(sec_size .data)
BSS=$(sec_size .bss)
# Survives soft-reset (not zeroed); currently the signal-lost boot cookie
NOINIT=$(sec_size .noinit)
HEAPSTACK=$(sec_size ._user_heap_stack)

# RX image in flash: code + const + vectab + filename + init arrays +
# the flash load image of .data
FLASH_USED=$((ISR + TEXT + RODATA + INITA + FINIA + FILE_NAME_SZ + DATA))
# RAM at runtime: .data + .bss + .noinit + heap/stack reservation
RAM_USED=$((DATA + BSS + NOINIT + HEAPSTACK))

FLASH_MAX=$((FLASH_CAPACITY * FLASH_MAX_PCT / 100))
RAM_MAX=$((RAM_CAPACITY * RAM_MAX_PCT / 100))

flash_pct=$(awk -v u="$FLASH_USED" -v c="$FLASH_CAPACITY" 'BEGIN{printf "%.2f", 100*u/c}')
ram_pct=$(awk -v u="$RAM_USED" -v c="$RAM_CAPACITY" 'BEGIN{printf "%.2f", 100*u/c}')

echo "=== size check: $ELF ==="
echo "  .text=$TEXT .rodata=$RODATA .data=$DATA .bss=$BSS .noinit=$NOINIT heap/stack=$HEAPSTACK"
echo "  FLASH used=$FLASH_USED / $FLASH_CAPACITY (${flash_pct}%)  limit=${FLASH_MAX} (${FLASH_MAX_PCT}%)"
echo "  RAM   used=$RAM_USED / $RAM_CAPACITY (${ram_pct}%)  limit=${RAM_MAX} (${RAM_MAX_PCT}%)"

rc=0
if [ "$FLASH_USED" -gt "$FLASH_MAX" ]; then
  echo "FAIL: flash $FLASH_USED > limit $FLASH_MAX" >&2
  rc=1
fi
if [ "$RAM_USED" -gt "$RAM_MAX" ]; then
  echo "FAIL: RAM $RAM_USED > limit $RAM_MAX" >&2
  rc=1
fi
if [ "$rc" -eq 0 ]; then
  echo "size-check-ark: PASS"
fi
exit "$rc"
