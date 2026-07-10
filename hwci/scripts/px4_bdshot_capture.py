#!/usr/bin/env python3
"""SETUP B only: capture PX4 ESC_STATUS over USB (BDShot eRPM / telem).

Not for SETUP A (Flight Stand). See docs/BENCH_SETUPS.md.

Intended setup (ARK FPV + PX4 BDShot):
  * ARK FPV USB → host (/dev/ttyACM*)
  * Actuator output = BDShot300/600 with EDT (if available)
  * Optional: KISS/serial ESC telem on a UART (publishes into same esc_status)
  * Optional second terminal: MAVLink shell (nsh) for `dshot` / `listener`

Examples:
  # Discover which ACM is PX4
  ./px4_bdshot_capture.py --port /dev/ttyACM0 --discover 3

  # Log 30 s while you motor-test from QGC / nsh
  ./px4_bdshot_capture.py --port /dev/ttyACM0 --duration 30 -o /tmp/bdshot_px4.csv

  # Request ESC_INFO once per second (serial telem / some ESCs)
  ./px4_bdshot_capture.py --port /dev/ttyACM0 --esc-info --duration 20

CSV columns are host wall-clock + whatever ESC_STATUS / ESC_INFO fields
pymavlink exposes for this dialect. Correlate later with SWD `perf_dshot_*`
from a simultaneous hwci perf poll if the ST-Link is attached.
"""
from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path


def _connect(port: str, baud: int):
    from pymavlink import mavutil

    # USB CDC: baud is ignored by the device but required by the API.
    m = mavutil.mavlink_connection(port, baud=baud, source_system=255)
    return m


def discover(port: str, baud: int, seconds: float) -> int:
    m = _connect(port, baud)
    t0 = time.time()
    n = 0
    types: dict[str, int] = {}
    print(f"listening on {port} for {seconds:.1f}s …", flush=True)
    while time.time() - t0 < seconds:
        msg = m.recv_match(blocking=True, timeout=0.25)
        if msg is None:
            continue
        n += 1
        t = msg.get_type()
        types[t] = types.get(t, 0) + 1
        if t == "HEARTBEAT":
            print(
                f"  HEARTBEAT sys={msg.get_srcSystem()} "
                f"comp={msg.get_srcComponent()} "
                f"autopilot={getattr(msg, 'autopilot', '?')} "
                f"type={getattr(msg, 'type', '?')}",
                flush=True,
            )
    print(f"messages={n} types={dict(sorted(types.items(), key=lambda kv: -kv[1])[:12])}")
    return 0 if n else 1


def _esc_status_rows(msg) -> list[dict]:
    """Flatten ESC_STATUS into one row per reported ESC index."""
    rows = []
    # Dialect fields vary slightly; tolerate missing attrs.
    count = int(getattr(msg, "count", 0) or 0)
    # Some builds always fill 4/8 slots; use count when sane.
    n = count if 0 < count <= 8 else 8
    t = time.time()
    for i in range(n):
        rpm = _arr(msg, "rpm", i)
        volt = _arr(msg, "voltage", i)
        cur = _arr(msg, "current", i)
        temp = _arr(msg, "temperature", i)
        # Skip completely empty slots
        if rpm is None and volt is None and cur is None:
            continue
        if (rpm or 0) == 0 and (volt or 0) == 0 and (cur or 0) == 0 and i >= max(count, 1):
            continue
        rows.append({
            "t": round(t, 6),
            "msg": "ESC_STATUS",
            "index": i,
            "count": count,
            "rpm": rpm,
            "voltage": volt,
            "current": cur,
            "temperature": temp,
            "error_count": _arr(msg, "error_count", i),
        })
    if not rows:
        # Still record a heartbeat-like status sample so silence is visible
        rows.append({
            "t": round(t, 6),
            "msg": "ESC_STATUS",
            "index": -1,
            "count": count,
            "rpm": None,
            "voltage": None,
            "current": None,
            "temperature": None,
            "error_count": None,
        })
    return rows


def _arr(msg, name: str, i: int):
    v = getattr(msg, name, None)
    if v is None:
        return None
    if isinstance(v, (list, tuple)):
        return v[i] if i < len(v) else None
    return v


def _esc_info_row(msg) -> dict:
    t = time.time()
    return {
        "t": round(t, 6),
        "msg": "ESC_INFO",
        "index": getattr(msg, "index", None),
        "count": getattr(msg, "count", None),
        "rpm": None,
        "voltage": getattr(msg, "voltage", None),
        "current": None,
        "temperature": getattr(msg, "temperature", None),
        "error_count": getattr(msg, "error_count", None),
        "info": str(msg),
    }


def capture(port: str, baud: int, duration: float, out: Path,
            esc_info: bool, rate_hz: float) -> int:
    m = _connect(port, baud)
    print(f"waiting for HEARTBEAT on {port} …", flush=True)
    m.wait_heartbeat(timeout=10)
    print(
        f"connected sys={m.target_system} comp={m.target_component}",
        flush=True,
    )

    # Prefer onboard ESC telemetry stream if the dialect supports the request.
    try:
        m.mav.command_long_send(
            m.target_system, m.target_component,
            511,  # MAV_CMD_SET_MESSAGE_INTERVAL
            0,
            291,  # ESC_STATUS (common dialect; ignore if unsupported)
            1e6 / max(rate_hz, 1.0),  # us interval
            0, 0, 0, 0, 0,
        )
    except Exception as e:
        print(f"note: SET_MESSAGE_INTERVAL ESC_STATUS failed: {e}", flush=True)

    fields = [
        "t", "msg", "index", "count", "rpm", "voltage", "current",
        "temperature", "error_count", "info",
    ]
    out.parent.mkdir(parents=True, exist_ok=True)
    n_status = n_info = n_other = 0
    t0 = time.time()
    next_info = t0
    with out.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        print(f"logging → {out} for {duration:.1f}s (Ctrl-C to stop)", flush=True)
        try:
            while time.time() - t0 < duration:
                now = time.time()
                if esc_info and now >= next_info:
                    next_info = now + 1.0
                    try:
                        # MAV_CMD_REQUEST_MESSAGE ESC_INFO (290) — best-effort
                        m.mav.command_long_send(
                            m.target_system, m.target_component,
                            512, 0, 290, 0, 0, 0, 0, 0, 0,
                        )
                    except Exception:
                        pass
                msg = m.recv_match(
                    type=["ESC_STATUS", "ESC_INFO", "HIGHRES_IMU", "HEARTBEAT"],
                    blocking=True, timeout=0.2,
                )
                if msg is None:
                    continue
                t = msg.get_type()
                if t == "ESC_STATUS":
                    for row in _esc_status_rows(msg):
                        w.writerow(row)
                        n_status += 1
                    fh.flush()
                elif t == "ESC_INFO":
                    w.writerow(_esc_info_row(msg))
                    n_info += 1
                    fh.flush()
                elif t == "HEARTBEAT":
                    n_other += 1
        except KeyboardInterrupt:
            print("\nstopped by user", flush=True)

    print(
        f"done: ESC_STATUS samples={n_status} ESC_INFO={n_info} "
        f"other={n_other} file={out}",
        flush=True,
    )
    if n_status == 0:
        print(
            "WARNING: no ESC_STATUS received.\n"
            "  - Confirm Actuators UI uses BDShot* (not plain DShot)\n"
            "  - Spin a motor (QGC Motor Test or nsh: dshot/actuator test)\n"
            "  - Serial telem UART configured if relying on KISS wire\n"
            "  - Try: mavlink shell → listener esc_status",
            flush=True,
        )
        return 2
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyACM0",
                    help="PX4 USB CDC port (default /dev/ttyACM0)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--discover", type=float, default=0,
                    help="only listen N seconds and print message types")
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("-o", "--output", type=Path,
                    default=Path("runs/bdshot_px4_capture.csv"))
    ap.add_argument("--esc-info", action="store_true",
                    help="periodically request ESC_INFO")
    ap.add_argument("--rate", type=float, default=50.0,
                    help="requested ESC_STATUS rate Hz (best-effort)")
    args = ap.parse_args(argv)

    if args.discover:
        return discover(args.port, args.baud, args.discover)
    return capture(args.port, args.baud, args.duration, args.output,
                   args.esc_info, args.rate)


if __name__ == "__main__":
    sys.exit(main())
