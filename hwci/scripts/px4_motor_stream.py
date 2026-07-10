#!/usr/bin/env python3
"""Drive PX4 ACTUATOR_TEST for free-run BDShot testing (PSU-safe).

Important behaviour (ARK FPV + PX4 1.18 + bench supply):

* ``MAV_CMD_ACTUATOR_TEST`` (310) works; ``DO_MOTOR_TEST`` (209) is unsupported.
* Each test is only effective for ~3 s; if it expires the FC drops the output
  **abruptly**. That can regenerate / spike and put a current-limited bench
  supply into **fault mode**. Always re-fire before timeout and **ramp down**.
* Do **not** spam COMMAND_LONG at 10–50 Hz (servo looks live, motor dies).
  Re-fire about every ``--refresh`` seconds (default 2.0) with ``--timeout 3``.
* Never jump high throttle → 0. Use a multi-step ramp-down at the end.

Example:

  ./px4_motor_stream.py --port /dev/ttyACM2 \\
      --steps 0.15:6,0.30:6,0.50:6 --refresh 2.0 --timeout 3.0 \\
      --ramp-down 4
"""
from __future__ import annotations

import argparse
import csv
import sys
import threading
import time
from pathlib import Path


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default='/dev/ttyACM2')
    ap.add_argument('--func', type=int, default=1,
                    help='ACTUATOR_OUTPUT_FUNCTION (Motor1=1)')
    ap.add_argument('--steps', default='0.15:6,0.30:6,0.50:6',
                    help='comma list of value:seconds (no hard zero between)')
    ap.add_argument('--refresh', type=float, default=1.5,
                    help='seconds between re-fire (must be well under ~3 s cap; '
                         'default 1.5 — late refresh lets PX4 hard-cut and can '
                         'fault the bench PSU)')
    ap.add_argument('--timeout', type=float, default=3.0,
                    help='ACTUATOR_TEST timeout (PX4 ~3 s effective cap)')
    ap.add_argument('--step-slew', type=float, default=1.0,
                    help='seconds to slew between consecutive step values')
    ap.add_argument('--ramp-down', type=float, default=6.0,
                    help='seconds for final smooth ramp to zero (PSU-safe stop)')
    ap.add_argument('--ramp-steps', type=int, default=12,
                    help='number of levels in final ramp-down')
    ap.add_argument('-o', '--output', type=Path, default=None)
    args = ap.parse_args()

    # Effective hold is ~2.8–3.0 s; refresh must leave margin or the FC drops
    # the line hard (regen / current spike → PSU fault).
    if args.refresh > args.timeout - 0.8:
        print(
            f'error: --refresh {args.refresh} too close to --timeout '
            f'{args.timeout}; use refresh <= timeout-0.8 (e.g. 1.5 / 3.0) '
            f'or the motor hard-stops and can fault the PSU',
            file=sys.stderr)
        return 2

    from pymavlink import mavutil

    steps: list[tuple[float, float]] = []
    for part in args.steps.split(','):
        part = part.strip()
        if not part:
            continue
        v, s = part.split(':')
        steps.append((float(v), float(s)))
    if not steps:
        print('error: no --steps', file=sys.stderr)
        return 2

    out = args.output or Path(
        f"runs/bdshot_sustain_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    out.parent.mkdir(parents=True, exist_ok=True)

    m = mavutil.mavlink_connection(args.port, baud=115200, source_system=255)
    print(f'waiting heartbeat on {args.port}…', flush=True)
    m.wait_heartbeat(timeout=10)
    sysid, comp = m.target_system, m.target_component
    print(f'connected sys={sysid} comp={comp}', flush=True)
    ml = mavutil.mavlink

    m.mav.command_long_send(sysid, comp, ml.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                            291, 20000, 0, 0, 0, 0, 0)
    m.mav.command_long_send(sysid, comp, ml.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                            36, 50000, 0, 0, 0, 0, 0)

    stop = threading.Event()
    rows: list[dict] = []
    lock = threading.Lock()
    latest = {'rpm': 0, 'v': 0.0, 'i': 0.0, 'servo': None, 'cmd': 0.0}
    current_cmd = 0.0

    def reader() -> None:
        while not stop.is_set():
            msg = m.recv_match(
                type=['ESC_STATUS', 'SERVO_OUTPUT_RAW', 'STATUSTEXT'],
                blocking=True, timeout=0.2)
            if msg is None:
                continue
            t = time.time()
            typ = msg.get_type()
            if typ == 'STATUSTEXT':
                print(f'  TEXT: {msg.text}', flush=True)
            elif typ == 'ESC_STATUS':
                rpm = msg.rpm[0] if msg.rpm else 0
                v = msg.voltage[0] if msg.voltage else 0.0
                c = msg.current[0] if msg.current else 0.0
                latest.update(rpm=rpm, v=v, i=c)
                with lock:
                    rows.append(dict(
                        t=t, msg='ESC_STATUS', rpm=rpm, voltage=v, current=c,
                        servo=str(latest.get('servo') or ''), cmd=latest['cmd']))
            elif typ == 'SERVO_OUTPUT_RAW':
                s = [getattr(msg, f'servo{i}_raw', 0) for i in range(1, 5)]
                latest['servo'] = s
                with lock:
                    rows.append(dict(
                        t=t, msg='SERVO', rpm='', voltage='', current='',
                        servo=str(s), cmd=latest['cmd']))

    threading.Thread(target=reader, daemon=True).start()
    time.sleep(0.3)

    def fire(value: float, timeout: float | None = None) -> None:
        nonlocal current_cmd
        current_cmd = float(value)
        latest['cmd'] = current_cmd
        to = args.timeout if timeout is None else timeout
        m.mav.command_long_send(
            sysid, comp, ml.MAV_CMD_ACTUATOR_TEST, 0,
            float(value), float(to), 0, 0, float(args.func), 0, 0)

    def sustain(value: float, seconds: float, label: str) -> None:
        """Hold a value, re-firing before ACTUATOR_TEST expires (no hard stop)."""
        print(
            f'>>> SUSTAIN func={args.func} value={value:.3f} for {seconds:.1f}s '
            f'refresh={args.refresh}s ({label})',
            flush=True)
        t0 = time.time()
        next_fire = t0
        last_print = t0
        while time.time() - t0 < seconds:
            now = time.time()
            if now >= next_fire:
                fire(value)
                next_fire = now + args.refresh
            if now - last_print >= 1.0:
                last_print = now
                print(
                    f'  t+{now - t0:4.1f}s cmd={value:.3f} rpm={latest["rpm"]} '
                    f'V={latest["v"]:.2f} I={latest["i"]:.3f} '
                    f'servo={latest.get("servo")}',
                    flush=True)
            time.sleep(0.05)

    def slew(from_v: float, to_v: float, seconds: float, label: str) -> None:
        """Linear slew; keeps re-firing so the timeout never hard-cuts mid-slew."""
        if seconds <= 0 or abs(to_v - from_v) < 1e-6:
            return
        n = max(2, int(seconds / max(args.refresh * 0.5, 0.25)))
        print(
            f'>>> SLEW {from_v:.3f} -> {to_v:.3f} over {seconds:.1f}s '
            f'({n} pts, {label})',
            flush=True)
        dt = seconds / n
        for i in range(1, n + 1):
            v = from_v + (to_v - from_v) * (i / n)
            # hold each intermediate with continuous refresh for dt
            sustain(v, dt, f'slew {i}/{n}')

    def ramp_down_safe(from_v: float) -> None:
        """PSU-safe stop: multi-step descent, never jump to zero from high duty.

        Hard ACTUATOR_TEST expiry or cmd→0 from mid/high throttle has put the
        3 A bench supply into fault mode. Always descend gradually while still
        refreshing so the ~3 s timeout never wins.
        """
        if from_v <= 0.02:
            # already essentially off — one gentle zero is OK
            fire(0.0, timeout=1.0)
            time.sleep(0.5)
            return
        print(
            f'>>> RAMP-DOWN from {from_v:.3f} over {args.ramp_down:.1f}s '
            f'({args.ramp_steps} steps) — avoid PSU fault',
            flush=True)
        # Geometric-ish descent spends more time at low duty (less regen shock)
        levels = []
        for i in range(1, args.ramp_steps + 1):
            # ease-out: slow near zero
            frac = 1.0 - (i / args.ramp_steps) ** 1.5
            levels.append(max(0.0, from_v * frac))
        levels[-1] = 0.0
        seg = args.ramp_down / len(levels)
        prev = from_v
        for v in levels:
            # keep refreshing during each segment
            sustain(v, max(seg, 0.35), f'ramp {v:.3f}')
            prev = v
        # linger at zero with refresh so we don't "expire" into a glitch
        sustain(0.0, 1.0, 'park zero')

    try:
        prev = 0.0
        for val, dur in steps:
            if abs(val - prev) > 0.02 and args.step_slew > 0:
                slew(prev, val, args.step_slew, 'step transition')
            sustain(val, dur, f'throttle {val}')
            prev = val
        ramp_down_safe(prev)
    except KeyboardInterrupt:
        print('\ninterrupted — soft ramp-down', flush=True)
        ramp_down_safe(current_cmd)
    except Exception:
        print('\nerror — soft ramp-down', flush=True)
        try:
            ramp_down_safe(current_cmd)
        except Exception:
            pass
        raise
    finally:
        # only a soft zero if somehow still commanding; do not hard-cut from high
        if current_cmd > 0.05:
            try:
                ramp_down_safe(current_cmd)
            except Exception:
                pass
        else:
            fire(0.0, timeout=0.3)
            time.sleep(0.2)
        stop.set()
        time.sleep(0.2)

    fields = ['t', 'msg', 'rpm', 'voltage', 'current', 'servo', 'cmd']
    with out.open('w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=fields)
        w.writeheader()
        with lock:
            w.writerows(rows)

    st = [r for r in rows if r['msg'] == 'ESC_STATUS']
    rpms = [float(r['rpm']) for r in st if r['rpm'] != '']
    print('==== SUMMARY ====', flush=True)
    print(f'file={out} esc_status={len(st)}', flush=True)
    if rpms:
        print(
            f'rpm min={min(rpms):.0f} max={max(rpms):.0f} '
            f'>1000={sum(1 for x in rpms if x > 1000)}/{len(rpms)}',
            flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
