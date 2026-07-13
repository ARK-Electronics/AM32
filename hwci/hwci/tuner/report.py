"""Tune-session report markdown builders, pilot card, and campaign table."""
from __future__ import annotations

import json
from pathlib import Path
from typing import Iterable

from .. import report as reportmod
from ..settings import Settings, resolve_field


def settings_rows(base: Settings, best: Settings, parameters: dict,
                  offsets: dict[str, int]) -> list[dict]:
    names = set(base.describe()) | set(best.describe()) | set(parameters)

    def field_offset(name: str) -> int:
        return resolve_field(name, offsets.get(name)).offset

    rows = []
    for name in sorted(names, key=field_offset):
        offset = offsets.get(name)
        default = base.get(name, offset)
        chosen = best.get(name, offset)
        rows.append({
            "setting": name,
            "offset": field_offset(name),
            "default": default,
            "best": chosen,
            "changed": default != chosen,
        })
    return rows


def diff_md(base: Settings, best: Settings) -> str:
    out = ["# Settings diff: default -> best\n",
           "| setting | default | best |", "|---|---|---|"]
    diff = base.diff(best)
    if not diff:
        out.append("| (no change - default settings won) | | |")
    for name, a, b in diff:
        out.append(f"| {name} | {a} | {b} |")
    out.append("")
    return "\n".join(out)


def report_md(manifest: dict, result: dict, settings_rows_list: list[dict],
              out_dir: Path) -> str:
    m = manifest
    out = [f"# AM32 auto-tune report: {m['spec_name']}\n"]
    out.append(f"- **mode**: {m['mode']}")
    out.append(f"- **git_sha**: {m['git_sha']}")
    out.append(f"- **eeprom_address**: 0x{m['eeprom_address']:08x}")
    out.append(f"- **battery_cells**: {m['battery_cells']}")
    out.append(f"- **jitter reference (default)**: "
               f"{m['jitter_reference']} %")
    out.append(f"- **pack swaps**: {len(m['pack_events'])}")
    out.append("")
    out.append("## Verdict\n")
    out.append(f"- winner: `{result['winner_overrides'] or '{} (default)'}`")
    thr = result.get("min_delta_threshold")
    thr_s = f", min-Δ threshold {thr} g/W" if thr is not None else ""
    out.append(f"- confirmed: **{result['confirmed']}** "
               f"(median paired delta {result['median_paired_delta']} g/W "
               f"over {len(result['paired_deltas'])} ABBA pairs"
               f"{thr_s}, "
               f"{result['winner_constraint_failures']} winner "
               "constraint failures)")
    if result.get("startup") is not None:
        st = result["startup"]
        out.append(f"- startup: {st['failed']}/{st['cycles']} failed")
    if result.get("high_throttle") is not None:
        ht = result["high_throttle"]
        thr_ht = ht.get("throttle")
        if ht.get("ok"):
            out.append(f"- high-throttle @{thr_ht}: pass")
        else:
            fails = ht.get("disqualified") or ["failed"]
            out.append(f"- high-throttle @{thr_ht}: FAIL "
                       f"({'; '.join(fails)})")
    out.append("\n## Full settings (default -> best)\n")
    out.append("| setting | offset | default | best | changed |")
    out.append("|---|---:|---:|---:|---|")
    for row in settings_rows_list:
        changed = "yes" if row["changed"] else ""
        out.append(f"| `{row['setting']}` | {row['offset']} | "
                   f"{row['default']} | {row['best']} | {changed} |")
    impact = reportmod.render_tune_settings_impact_markdown(
        m, settings_rows_list)
    if impact:
        out.append("")
        out.append(impact)
    out.append("\n## Stages\n")
    out.append("| stage | winner | score | efficiency argmax | argmax score "
               "| reason |")
    out.append("|---|---|---:|---|---:|---|")
    for name, s in m["stages"].items():
        argmax = s.get("efficiency_argmax")
        argmax_s = s.get("efficiency_argmax_score")
        reason = s.get("winner_reason") or ""
        # Highlight when tie-break discarded pure efficiency max.
        note = reason
        if (argmax is not None and s.get("winner") is not None
                and argmax != s.get("winner")):
            note = reason or "noise_floor_tiebreak"
        out.append(
            f"| {name} | `{s['winner']}` | {s['winner_score']} | "
            f"`{argmax}` | {argmax_s} | {note} |")
    out.append("\n## Trials\n")
    out.append("| # | stage | kind | overrides | raw g/W | norm g/W "
               "| disqualified |")
    out.append("|---|---|---|---|---|---|---|")
    for e in m["trials"]:
        dq = "; ".join(e["disqualified"]) if e.get("disqualified") else ""
        if e.get("discarded"):
            dq = (dq + " " if dq else "") + "(discarded: pack swap)"
        out.append(f"| {e['index']} | {e['stage']} | {e['kind']} | "
                   f"`{e['overrides']}` | {e['score_raw']} | "
                   f"{e['score_norm']} | {dq} |")
    out.append("")
    out.append(reportmod.render_tune_raw_markdown(out_dir, m))
    return "\n".join(out)


def pilot_card_payload(manifest: dict, result: dict,
                       settings_rows_list: list[dict],
                       base: Settings, best: Settings,
                       session_dir: Path | None = None) -> dict:
    """Structured one-page summary for humans and campaign aggregation."""
    changed = [
        {"setting": r["setting"], "default": r["default"], "best": r["best"]}
        for r in settings_rows_list if r["changed"]
    ]
    stage_notes = []
    for name, s in (manifest.get("stages") or {}).items():
        if (s.get("efficiency_argmax") is not None
                and s.get("winner") is not None
                and s.get("efficiency_argmax") != s.get("winner")):
            stage_notes.append({
                "stage": name,
                "winner": s.get("winner"),
                "efficiency_argmax": s.get("efficiency_argmax"),
                "winner_reason": s.get("winner_reason"),
                "winner_score": s.get("winner_score"),
                "efficiency_argmax_score": s.get("efficiency_argmax_score"),
            })
    ht = result.get("high_throttle")
    st = result.get("startup")
    return {
        "session_dir": str(session_dir) if session_dir else None,
        "spec_name": manifest.get("spec_name"),
        "mode": manifest.get("mode"),
        "git_sha": manifest.get("git_sha"),
        "battery_cells": manifest.get("battery_cells"),
        "confirmed": bool(result.get("confirmed")),
        "winner_overrides": result.get("winner_overrides") or {},
        "median_paired_delta": result.get("median_paired_delta"),
        "min_delta_threshold": result.get("min_delta_threshold"),
        "paired_deltas": result.get("paired_deltas") or [],
        "abba_blocks": result.get("abba_blocks"),
        "startup_failed": None if st is None else st.get("failed"),
        "startup_cycles": None if st is None else st.get("cycles"),
        "high_throttle_ok": None if ht is None else ht.get("ok"),
        "high_throttle": None if ht is None else ht.get("throttle"),
        "pack_swaps": len(manifest.get("pack_events") or []),
        "settings_changed": changed,
        "settings_diff": [
            {"name": n, "default": a, "best": b}
            for n, a, b in base.diff(best)
        ],
        "stage_tiebreaks": stage_notes,
        "n_trials": len(manifest.get("trials") or []),
    }


def pilot_card_md(payload: dict) -> str:
    """Human-readable one-page pilot card."""
    conf = "CONFIRMED" if payload.get("confirmed") else "NOT CONFIRMED"
    lines = [
        f"# Pilot card: {payload.get('spec_name')}\n",
        f"**{conf}** — flash `{ 'best_settings.bin' if payload.get('confirmed') else 'base / defaults' }`\n",
        f"- mode: `{payload.get('mode')}`  git: `{payload.get('git_sha')}`  "
        f"cells: {payload.get('battery_cells')}",
        f"- winner: `{payload.get('winner_overrides') or '{}'}`",
        f"- median paired Δ: **{payload.get('median_paired_delta')}** g/W "
        f"(threshold {payload.get('min_delta_threshold')} g/W, "
        f"{len(payload.get('paired_deltas') or [])} pairs, "
        f"{payload.get('abba_blocks')} ABBA block(s))",
    ]
    if payload.get("startup_cycles") is not None:
        lines.append(
            f"- startup: {payload.get('startup_failed')}/"
            f"{payload.get('startup_cycles')} failed")
    if payload.get("high_throttle") is not None:
        ok = "pass" if payload.get("high_throttle_ok") else "FAIL"
        lines.append(f"- high-throttle @{payload.get('high_throttle')}: {ok}")
    lines.append(f"- pack swaps: {payload.get('pack_swaps')}  "
                 f"trials: {payload.get('n_trials')}")
    lines.append("\n## Settings to flash\n")
    changed = payload.get("settings_changed") or []
    if not changed:
        lines.append("_No changes — keep firmware defaults._\n")
    else:
        lines.append("| setting | default | best |")
        lines.append("|---|---:|---:|")
        for r in changed:
            lines.append(
                f"| `{r['setting']}` | {r['default']} | {r['best']} |")
        lines.append("")
    notes = payload.get("stage_tiebreaks") or []
    if notes:
        lines.append("## Efficiency argmax ≠ chosen\n")
        lines.append("| stage | chosen | argmax | reason |")
        lines.append("|---|---|---|---|")
        for n in notes:
            lines.append(
                f"| {n['stage']} | `{n['winner']}` | "
                f"`{n['efficiency_argmax']}` | {n.get('winner_reason')} |")
        lines.append("")
    lines.append(
        "_Full detail: `report.md`. Campaign rollup: "
        "`hwci campaign <session-dirs...>`._\n")
    return "\n".join(lines)


def write_pilot_card(out_dir: Path, manifest: dict, result: dict,
                     settings_rows_list: list[dict],
                     base: Settings, best: Settings
                     ) -> tuple[Path, Path]:
    """Write pilot_card.md + pilot_card.json; return their paths."""
    out_dir = Path(out_dir)
    payload = pilot_card_payload(
        manifest, result, settings_rows_list, base, best, out_dir)
    md_path = out_dir / "pilot_card.md"
    json_path = out_dir / "pilot_card.json"
    md_path.write_text(pilot_card_md(payload))
    json_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    return md_path, json_path


def load_pilot_card(session_dir: Path) -> dict | None:
    p = Path(session_dir) / "pilot_card.json"
    if not p.exists():
        # Fall back to reconstructing from manifest + result if present.
        man = Path(session_dir) / "manifest.json"
        if not man.exists():
            return None
        m = json.loads(man.read_text())
        result = m.get("result")
        if not result:
            return None
        return {
            "session_dir": str(session_dir),
            "spec_name": m.get("spec_name"),
            "mode": m.get("mode"),
            "git_sha": m.get("git_sha"),
            "battery_cells": m.get("battery_cells"),
            "confirmed": bool(result.get("confirmed")),
            "winner_overrides": result.get("winner_overrides") or {},
            "median_paired_delta": result.get("median_paired_delta"),
            "min_delta_threshold": result.get("min_delta_threshold"),
            "paired_deltas": result.get("paired_deltas") or [],
            "abba_blocks": result.get("abba_blocks"),
            "pack_swaps": len(m.get("pack_events") or []),
            "n_trials": len(m.get("trials") or []),
            "settings_changed": [],
            "stage_tiebreaks": [],
        }
    data = json.loads(p.read_text())
    data.setdefault("session_dir", str(session_dir))
    return data


def campaign_table_md(cards: Iterable[dict]) -> str:
    """Aggregate multiple pilot cards into a campaign comparison table."""
    cards = list(cards)
    lines = [
        "# Tune campaign summary\n",
        f"Sessions: **{len(cards)}** "
        f"({sum(1 for c in cards if c.get('confirmed'))} confirmed)\n",
        "| session | spec | confirmed | Δ g/W | winner | cells | trials |",
        "|---|---|---|---:|---|---:|---:|",
    ]
    for c in cards:
        sess = Path(c.get("session_dir") or "?").name
        conf = "yes" if c.get("confirmed") else "no"
        delta = c.get("median_paired_delta")
        delta_s = "" if delta is None else f"{delta:.4f}"
        winner = c.get("winner_overrides") or {}
        # Compact winner: only non-empty overrides
        if isinstance(winner, dict) and winner:
            w = ",".join(f"{k}={v}" for k, v in sorted(winner.items()))
        else:
            w = "(default)"
        lines.append(
            f"| `{sess}` | {c.get('spec_name')} | {conf} | {delta_s} | "
            f"`{w}` | {c.get('battery_cells')} | {c.get('n_trials')} |")
    lines.append("")
    # Changed settings union
    lines.append("## Changed settings (confirmed sessions)\n")
    any_changed = False
    for c in cards:
        if not c.get("confirmed"):
            continue
        ch = c.get("settings_changed") or c.get("settings_diff") or []
        if not ch:
            continue
        any_changed = True
        sess = Path(c.get("session_dir") or "?").name
        lines.append(f"### `{sess}`\n")
        lines.append("| setting | default | best |")
        lines.append("|---|---:|---:|")
        for r in ch:
            name = r.get("setting") or r.get("name")
            lines.append(
                f"| `{name}` | {r.get('default')} | "
                f"{r.get('best')} |")
        lines.append("")
    if not any_changed:
        lines.append("_No confirmed sessions with setting changes._\n")
    return "\n".join(lines)
