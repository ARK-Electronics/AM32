#!/usr/bin/env python3
"""Convert a bootloader .bin into a C header with a const uint8_t array.

Pads to --pad-to bytes (default 4096) with 0xFF so the image fills the AM32
bootloader region. Ensures even length for halfword flash programming.
"""
from __future__ import annotations

import argparse
from pathlib import Path


def main() -> int:
	p = argparse.ArgumentParser(description=__doc__)
	p.add_argument("binfile", type=Path, help="bootloader .bin input")
	p.add_argument("header", type=Path, help="C header output path")
	p.add_argument("--array-name", default="bl_image", help="C array name")
	p.add_argument(
		"--pad-to",
		type=int,
		default=4096,
		help="pad image to this size with 0xFF (0 = no pad)",
	)
	args = p.parse_args()

	data = bytearray(args.binfile.read_bytes())
	if args.pad_to:
		if len(data) > args.pad_to:
			raise SystemExit(f"bin length {len(data)} exceeds --pad-to {args.pad_to}")
		if len(data) < args.pad_to:
			data.extend(b"\xff" * (args.pad_to - len(data)))
	if len(data) % 2:
		data.append(0xFF)

	guard = f"{args.array_name.upper()}_H"
	lines = [
		f"// Auto-generated from {args.binfile.name} ({len(data)} bytes)",
		f"// Regenerate: python3 scripts/gen_bl_header.py {args.binfile} {args.header}",
		"#pragma once",
		"#include <stdint.h>",
		"",
		f"static const uint8_t {args.array_name}[] = {{",
	]
	for i in range(0, len(data), 16):
		chunk = data[i : i + 16]
		hexes = ", ".join(f"0x{b:02X}" for b in chunk)
		comma = "," if i + 16 < len(data) else ""
		lines.append(f"\t{hexes}{comma}")
	lines.append("};")
	lines.append("")

	args.header.parent.mkdir(parents=True, exist_ok=True)
	args.header.write_text("\n".join(lines) + "\n")
	print(f"Wrote {args.header} ({len(data)} bytes, guard {guard})")
	return 0


if __name__ == "__main__":
	raise SystemExit(main())
