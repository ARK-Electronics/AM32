#!/usr/bin/env python3
"""
profile_swd.py - live readout of the PROFILE_ISR counters over SWD.

The F051 (Cortex-M0) has no DWT/SWO, so the profiling harness (Inc/profiling.h)
times each hot ISR with TIM17 as a 48 MHz cycle counter and accumulates the
results into RAM globals. This script reads those globals through a running
OpenOCD over its Tcl RPC port and turns them into per-ISR CPU load, average and
worst-case microseconds, and call rate.

Reads go through the debug access port and do NOT halt the core, so it is safe
to run while the motor is spinning (no commutation glitch, no watchdog reset).

Usage:
    1. Build with PROFILE_ISR enabled and flash the board.
    2. Start OpenOCD (its Tcl port 6666 is on by default):
         openocd -f Mcu/f051/openocd.cfg
       (or just leave your VS Code / Cortex-Debug session running)
    3. Run this against the matching ELF:
         scripts/profile_swd.py obj/AM32_ARK_4IN1_F051_2.20.elf

Options:
    --host/--port   OpenOCD Tcl RPC endpoint (default 127.0.0.1:6666)
    --interval      seconds between samples (default 1.0)
    --cpu-mhz       core clock for cycle->time conversion (default 48)
    --nm            path to nm (default arm-none-eabi-nm)

Requires OpenOCD >= 0.11 (for the `read_memory` command).
"""
import argparse
import re
import socket
import subprocess
import sys
import time

# enum order in Inc/profiling.h
LABELS = ["comp_zc(ADC1_COMP)", "20khz(TIM6)", "dshot(EXTI4_15)", "tim14"]
U32 = 0xFFFFFFFF


class OpenOcdTcl:
    """Minimal OpenOCD Tcl RPC client (commands terminated by 0x1a)."""
    SEP = b"\x1a"

    def __init__(self, host, port):
        self.sock = socket.create_connection((host, port), timeout=5)

    def cmd(self, command):
        self.sock.sendall(command.encode() + self.SEP)
        buf = bytearray()
        while not buf.endswith(self.SEP):
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("OpenOCD closed the connection")
            buf.extend(chunk)
        return buf[:-1].decode(errors="replace")

    def read_words(self, addr, width, count):
        # read_memory returns a space-separated list of decimal values
        out = self.cmd(f"read_memory 0x{addr:08x} {width} {count}")
        return [int(tok, 0) for tok in out.split()]

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


def resolve_symbols(nm, elf, names):
    """Return {name: (addr, size_bytes)} via `nm -S`."""
    try:
        out = subprocess.check_output([nm, "-S", "--radix=d", elf], text=True)
    except (OSError, subprocess.CalledProcessError) as e:
        sys.exit(f"failed to run {nm} on {elf}: {e}")
    syms = {}
    for line in out.splitlines():
        # "<addr> <size> <type> <name>"
        m = re.match(r"(\d+)\s+(\d+)\s+\S\s+(\S+)", line)
        if m and m.group(3) in names:
            syms[m.group(3)] = (int(m.group(1)), int(m.group(2)))
    missing = [n for n in names if n not in syms]
    if missing:
        sys.exit(f"symbols not found in {elf} (build with PROFILE_ISR?): {missing}")
    return syms


def main():
    ap = argparse.ArgumentParser(description="Live SWD readout of PROFILE_ISR counters.")
    ap.add_argument("elf", help="ELF built with PROFILE_ISR")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=6666)
    ap.add_argument("--interval", type=float, default=1.0)
    ap.add_argument("--cpu-mhz", type=float, default=48.0)
    ap.add_argument("--nm", default="arm-none-eabi-nm")
    args = ap.parse_args()

    cyc_per_us = args.cpu_mhz
    cpu_hz = args.cpu_mhz * 1e6

    syms = resolve_symbols(args.nm, args.elf, ["prof_cyc_max", "prof_cyc_sum", "prof_calls"])
    count = syms["prof_calls"][1] // 4 or len(LABELS)
    labels = (LABELS + [f"isr{i}" for i in range(len(LABELS), count)])[:count]

    try:
        ocd = OpenOcdTcl(args.host, args.port)
    except OSError as e:
        sys.exit(f"cannot reach OpenOCD Tcl port {args.host}:{args.port} ({e}); "
                 f"is OpenOCD running with `-f Mcu/f051/openocd.cfg`?")

    def sample():
        return (
            ocd.read_words(syms["prof_cyc_sum"][0], 32, count),
            ocd.read_words(syms["prof_calls"][0], 32, count),
            ocd.read_words(syms["prof_cyc_max"][0], 16, count),
        )

    print(f"# reading {count} ISR counters @ {args.cpu_mhz:g} MHz, every {args.interval:g}s "
          f"(Ctrl-C to stop)")
    prev_sum, prev_calls, _ = sample()
    prev_t = time.monotonic()
    try:
        while True:
            time.sleep(args.interval)
            cur_sum, cur_calls, cur_max = sample()
            now = time.monotonic()
            dt = now - prev_t

            print(f"\n=== dt={dt:.2f}s ===")
            print(f"{'ISR':22}{'rate(Hz)':>10}{'avg(us)':>9}{'max(us)':>9}{'CPU%':>8}")
            total = 0.0
            for i in range(count):
                dcalls = (cur_calls[i] - prev_calls[i]) & U32
                dsum = (cur_sum[i] - prev_sum[i]) & U32
                rate = dcalls / dt
                load = 100.0 * dsum / (dt * cpu_hz)
                avg_us = (dsum / dcalls) / cyc_per_us if dcalls else 0.0
                max_us = cur_max[i] / cyc_per_us
                total += load
                print(f"{labels[i]:22}{rate:10.0f}{avg_us:9.2f}{max_us:9.2f}{load:8.1f}")
            print(f"{'TOTAL (instrumented)':22}{'':>10}{'':>9}{'':>9}{total:8.1f}")

            prev_sum, prev_calls, prev_t = cur_sum, cur_calls, now
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        ocd.close()


if __name__ == "__main__":
    main()
