#!/usr/bin/env bash
# Static analysis for the ARK F051 control path (Src/*.c excluding DroneCAN).
#
# Usage:
#   ./scripts/cppcheck-ark.sh          # exit 1 on warning/error
#   make cppcheck
#
# Requires: cppcheck (apt install cppcheck / brew install cppcheck)
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

if ! command -v cppcheck >/dev/null 2>&1; then
  echo "error: cppcheck not found (install with: sudo apt-get install -y cppcheck)" >&2
  exit 127
fi

OUT_DIR="${OUT_DIR:-obj}"
mkdir -p "$OUT_DIR"
REPORT="$OUT_DIR/cppcheck-ark.txt"
SUPPRESSIONS="${ROOT}/scripts/cppcheck-suppressions.txt"

FILE_LIST="$(mktemp)"
trap 'rm -f "$FILE_LIST"' EXIT
# Control firmware only — not generated DroneCAN / libcanard.
find Src -maxdepth 1 -name '*.c' ! -name 'firmwareversion.c' | sort > "$FILE_LIST"
# Include firmwareversion if present (tiny)
if [ -f Src/firmwareversion.c ]; then
  echo Src/firmwareversion.c >> "$FILE_LIST"
fi

# --max-configs=1 is mandatory: Inc/targets.h has hundreds of board ifdefs;
# without it cppcheck expands an enormous config space and hangs.
#
# Fail the job on warnings/errors (not pure "style" nits). Style is still
# printed so dead branches (e.g. ARK dead-time clamp) show up in the log.
set +e
cppcheck \
  --enable=warning,performance,portability,style \
  --std=c11 \
  --max-configs=1 \
  --inline-suppr \
  --error-exitcode=1 \
  --suppressions-list="$SUPPRESSIONS" \
  -DARK_4IN1_F051 \
  -DSTM32F051x8 \
  -DSTMICRO \
  -DUSE_FULL_LL_DRIVER \
  -DHWCI_PERF \
  -D__GNUC__=11 \
  -D__ARM_ARCH_6M__=1 \
  -I Inc \
  -I Mcu/f051/Inc \
  -I Mcu/f051/Drivers/STM32F0xx_HAL_Driver/Inc \
  -I Mcu/f051/Drivers/CMSIS/Include \
  -I Mcu/f051/Drivers/CMSIS/Device/ST/STM32F0xx/Include \
  --file-list="$FILE_LIST" \
  2> "$REPORT"
rc=$?
set -e

# Pretty summary for CI logs
echo "=== cppcheck report ($REPORT) ==="
if [ -s "$REPORT" ]; then
  cat "$REPORT"
else
  echo "(no findings)"
fi

# Treat only error/warning lines as failures; style/performance stay informational
# if cppcheck returned non-zero solely due to style... Actually --error-exitcode=1
# fails on any enabled check including style. Filter:
hard=$(grep -cE ' error:| warning:' "$REPORT" 2>/dev/null || true)
style=$(grep -cE ' style:| performance:| portability:' "$REPORT" 2>/dev/null || true)
echo "=== summary: hard(error/warning)=$hard style/perf/port=$style cppcheck_rc=$rc ==="

if [ "${hard:-0}" -gt 0 ]; then
  echo "cppcheck: FAIL ($hard error/warning finding(s))" >&2
  exit 1
fi
echo "cppcheck: PASS (no error/warning; style/perf findings are advisory)"
exit 0
