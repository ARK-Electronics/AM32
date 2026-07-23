#!/usr/bin/env bash
# Verify the pinned xPack GNU Arm Embedded GCC is present and is GCC 15.x.
# Usage: check-arm-sdk.sh <ARM_SDK_PREFIX> <XPACK_GCC_VER>
# Example: check-arm-sdk.sh tools/linux/xpack-arm-none-eabi-gcc-15.2.1-1.1/bin/arm-none-eabi- 15.2.1-1.1
set -euo pipefail

PREFIX="${1:?ARM_SDK_PREFIX required}"
PIN_VER="${2:?XPACK_GCC_VER required}"

GCC="${PREFIX}gcc"
if [ ! -x "$GCC" ] && [ -f "${PREFIX}gcc.exe" ]; then
	GCC="${PREFIX}gcc.exe"
fi

if [ ! -x "$GCC" ] && [ ! -f "$GCC" ]; then
	echo "error: pinned xPack Arm GCC not found: ${PREFIX}gcc" >&2
	echo "       expected pin ${PIN_VER}" >&2
	echo "       install with:  make arm_sdk_install" >&2
	echo "       (do not use distro gcc-arm-none-eabi — CI and size gates require 15.x)" >&2
	exit 1
fi

# -dumpversion is "15.2.1"; --version first line has the xPack banner.
ver="$("$GCC" -dumpversion 2>/dev/null || true)"
case "$ver" in
15.*)
	;;
*)
	echo "error: expected GCC 15.x at $GCC, got '${ver:-unknown}'" >&2
	echo "       pin is ${PIN_VER}; remove stale tools and re-run: make arm_sdk_install" >&2
	"$GCC" --version 2>&1 | head -3 >&2 || true
	exit 1
	;;
esac

# Optional: confirm major.minor matches the pin when dumpversion is full (15.2.1).
pin_mm="${PIN_VER%%-*}" # 15.2.1-1.1 -> 15.2.1
if [ -n "$ver" ] && [ -n "$pin_mm" ] && [ "$ver" != "$pin_mm" ]; then
	# Accept 15.2.1 vs 15.2 style mismatches only if both are 15.x (already checked).
	case "$ver" in
	"${pin_mm}"*) ;;
	*)
		# dumpversion sometimes omits patch; only warn if major.minor diverge
		ver_mm=$(echo "$ver" | cut -d. -f1-2)
		pin_majmin=$(echo "$pin_mm" | cut -d. -f1-2)
		if [ "$ver_mm" != "$pin_majmin" ]; then
			echo "error: GCC at $GCC is $ver but pin is $PIN_VER" >&2
			exit 1
		fi
		;;
	esac
fi

echo "arm_sdk: $GCC (GCC $ver, pin $PIN_VER)"
