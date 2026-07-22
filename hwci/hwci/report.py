"""Render run metrics + regression comparison to Markdown (and optional plots)."""
from __future__ import annotations

import json
import statistics
import textwrap
from pathlib import Path


def _fmt(v) -> str:
    if v is None:
        return "-"
    if isinstance(v, float):
        return f"{v:.3f}" if abs(v) < 1000 else f"{v:.0f}"
    return str(v)


def render_markdown(metrics: dict, comparison: dict | None = None,
                    meta: dict | None = None) -> str:
    out: list[str] = []
    out.append("# AM32 Hardware-CI Report\n")
    if meta:
        out.append("## Run\n")
        for k in ("target", "profile", "mode", "git_sha", "firmware_version",
                  "motor", "prop", "pole_pairs", "timestamp", "aborted",
                  "perf_read_errors"):
            if k in meta and meta[k] is not None:
                out.append(f"- **{k}**: {meta[k]}")
        out.append("")

    s = metrics["summary"]
    smoke = metrics.get("smoke_gates")
    if smoke is not None or comparison is not None:
        smoke_ok = True if smoke is None else bool(smoke.get("passed"))
        base_ok = True if comparison is None else bool(comparison.get("passed"))
        verdict = "✅ PASS" if (smoke_ok and base_ok) else "❌ FAIL"
        out.append(f"## Verdict: {verdict}\n")

    out.append("## Summary\n")
    out.append("| metric | value |")
    out.append("|---|---|")
    for k, v in s.items():
        out.append(f"| {k} | {_fmt(v)} |")
    out.append("")

    pts = metrics.get("steady_points", [])
    if pts:
        out.append("## Steady-state operating points\n")
        cols = ["segment", "throttle", "rpm", "thrust_gf", "current_a",
                "voltage_v", "elec_power_w", "eff_gf_per_w",
                "ctrl_exec_us_max", "cpu_load_pct",
                "zc_jitter_pct", "zc_jitter_max_pct"]
        out.append("| " + " | ".join(cols) + " |")
        out.append("|" + "---|" * len(cols))
        for p in pts:
            out.append("| " + " | ".join(_fmt(p.get(c)) for c in cols) + " |")
        out.append("")

    st = metrics.get("startup", {})
    if st.get("attempts"):
        out.append("## Start attempts\n")
        out.append(f"- attempts: {st['attempts']}, **failures: {st['failures']}**")
        out.append(f"- time to running: mean {_fmt(st.get('time_to_run_ms_mean'))} ms, "
                   f"max {_fmt(st.get('time_to_run_ms_max'))} ms")
        fails = [a["segment"] for a in st.get("per_attempt", []) if not a["success"]]
        if fails:
            out.append(f"- failed segments: {', '.join(fails)}")
        out.append("")

    d = metrics.get("demag", {})
    out.append("## Demag / desync\n")
    out.append(f"- events: **{d.get('event_count', 0)}**")
    out.append(f"- bemf-timeout samples: {d.get('bemf_timeout_samples', 0)}")
    out.append(f"- commutation-spike samples: {d.get('comm_spike_samples', 0)}")
    out.append(f"- ESC-eRPM vs stand-RPM mismatch samples: "
               f"{d.get('esc_rpm_mismatch_samples', 0)}")
    out.append("")

    smoke = metrics.get("smoke_gates")
    if smoke is not None:
        out.append("## Smoke / health gates\n")
        out.append(f"- overall: **{'PASS' if smoke.get('passed') else 'FAIL'}**")
        out.append("")
        out.append("| check | current | limit | pass | detail |")
        out.append("|---|---|---|---|---|")
        for c in smoke.get("checks", []):
            mark = "✅" if c.get("pass") else "❌"
            out.append(
                f"| {c.get('name')} | {_fmt(c.get('current'))} | "
                f"{_fmt(c.get('limit'))} | {mark} | {c.get('detail', '')} |")
        out.append("")

    if comparison is not None:
        out.append("## Regression checks (vs baseline)\n")
        out.append("| check | baseline | current | pass | rule |")
        out.append("|---|---|---|---|---|")
        for c in comparison["checks"]:
            mark = "✅" if c["pass"] else "❌"
            out.append(f"| {c['name']} | {_fmt(c['baseline'])} | "
                       f"{_fmt(c['current'])} | {mark} | {c['note']} |")
        out.append("")

    return "\n".join(out)


def write_report(run_dir: str | Path, metrics: dict,
                 comparison: dict | None = None, meta: dict | None = None,
                 plots: bool = True) -> Path:
    run_dir = Path(run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)
    md = render_markdown(metrics, comparison, meta)
    report_path = run_dir / "report.md"
    report_path.write_text(md)
    if plots:
        try:
            _write_plots(run_dir, metrics)
        except Exception:
            pass  # plotting is best-effort / optional
    return report_path


# --------------------------------------------------------------------------
# Auto-tune PDF report
# --------------------------------------------------------------------------
def write_tune_pdf(out_dir: str | Path, manifest: dict, result: dict,
                   diff_rows: list | None = None,
                   settings_rows: list | None = None, *,
                   log=None) -> Path | None:
    """Render one auto-tune session to a multi-page PDF (``tune_report.pdf``).

    Best-effort: uses matplotlib's ``PdfPages`` so it needs nothing beyond the
    existing optional ``plot`` extra. Returns the PDF path on success, or
    ``None`` if matplotlib is unavailable / rendering fails (the Markdown
    report is written by the caller regardless).

    ``diff_rows`` is the ``Settings.diff`` output ``[(name, default, best)]``.
    ``settings_rows`` is the full default/best tunable setting table.
    """
    def _log(msg: str) -> None:
        if log is not None:
            log(msg)

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.backends.backend_pdf import PdfPages
    except Exception as e:  # matplotlib not installed -> skip, don't fail tune
        _log(f"PDF report skipped (matplotlib unavailable: {e}); "
             "install the 'plot' extra to enable it")
        return None

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    pdf_path = out_dir / "tune_report.pdf"
    try:
        with PdfPages(pdf_path) as pdf:
            _pdf_cover_page(plt, pdf, out_dir, manifest, result,
                           diff_rows or [], settings_rows)
            _pdf_efficiency_curve_page(plt, pdf, out_dir, manifest, result)
            _pdf_settings_pages(plt, pdf, settings_rows or [])
            _pdf_settings_impact_pages(plt, pdf, manifest, settings_rows or [])
            _pdf_progress_page(plt, pdf, manifest, result)
            _pdf_tables_pages(plt, pdf, manifest)
            _pdf_raw_data_pages(plt, pdf, out_dir, manifest)
    except Exception as e:
        _log(f"PDF report failed to render: {e}")
        try:
            pdf_path.unlink()
        except OSError:
            pass
        return None
    return pdf_path


_PAGE = (8.27, 11.69)        # A4 portrait, inches
_PAGE_LS = (11.69, 8.27)     # A4 landscape (wide tables)
_CONFIRMED_GREEN = "#1a7f37"
_DEFAULT_GRAY = "#57606a"
_BORDER_GRAY = "#d0d7de"
_HEADER_BG = "#f6f8fa"


def _wrap(v, width: int) -> str:
    s = _fmt(v) if not isinstance(v, str) else v
    return "\n".join(textwrap.wrap(s, width)) if len(s) > width else s


def _place_table(ax, rows: list, cols: list, widths: list,
                 fontsize: float = 8.0, line_h: float = 0.032,
                 dim_rows: set[int] | None = None) -> None:
    """Left-aligned table pinned to the top of ``ax`` with EXPLICIT column
    width fractions (auto_set_column_width lets wide tables overflow the
    page and clip - seen on real trial tables). Cell text may contain
    newlines (pre-wrapped); row heights follow the tallest cell.
    ``dim_rows`` (0-indexed into ``rows``, i.e. excluding the header) is
    rendered in grey - e.g. a setting the configurator itself greys out /
    disables under the current combination of settings."""
    ax.axis("off")
    if not rows:
        ax.text(0.5, 0.5, "(none)", ha="center", va="center",
                transform=ax.transAxes, color=_DEFAULT_GRAY)
        return
    tbl = ax.table(cellText=rows, colLabels=cols, cellLoc="left",
                   colLoc="left", loc="upper left", colWidths=widths)
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(fontsize)
    lines_in_row: dict[int, int] = {}
    for (r, _c), cell in tbl.get_celld().items():
        n = cell.get_text().get_text().count("\n") + 1
        lines_in_row[r] = max(lines_in_row.get(r, 1), n)
    for (r, _c), cell in tbl.get_celld().items():
        cell.set_height(line_h * (lines_in_row[r] + 0.6))
        cell.set_edgecolor(_BORDER_GRAY)
        cell.PAD = 0.02
        if r == 0:
            cell.set_facecolor(_HEADER_BG)
            cell.set_text_props(fontweight="bold")
        elif dim_rows and (r - 1) in dim_rows:
            cell.set_text_props(color=_DEFAULT_GRAY)


# Raw EEPROM byte -> the value shown in the AM32 web configurator, for
# direct transcription into that tool. Sources: Inc/eeprom.h + Src/main.c
# loadEEpromSettings() (advance_level's old(0-3)<->new(10-42) format
# conversion; new format subtracts 10 then advances in 0.9375 deg steps,
# i.e. degrees = (advance_level - 10) * 0.9375 - cross-checked against
# wiki.am32.ca's EEPROM format doc, which gives old-format raw 2 = 15
# degrees: old->new gives raw 26, (26-10)*0.9375 = 15, matching), and
# wiki.am32.ca's "ESC Settings Explained" + community docs for the
# configurator's field labels and units (PWM Frequency in kHz and
# max_ramp/Ramp Speed in %/ms are stored directly in those units already;
# Variable PWM's 0/1/2 map to the configurator's Fixed/Variable(Low-High)/
# By RPM options).
_CONFIGURATOR_LABELS = {
    "advance_level": "Motor Timing",
    "pwm_frequency": "PWM Frequency",
    "variable_pwm": "Variable PWM",
    "max_ramp": "Ramp Speed",
    "minimum_duty_cycle": "Minimum Duty Cycle",
    "startup_power": "Startup Power",
    "auto_advance": "Auto Timing Advance",
}
_VARIABLE_PWM_LABELS = {0: "Fixed", 1: "Variable (Low/High)", 2: "By RPM"}


def _configurator_value(name: str, raw) -> str | None:
    """The AM32 web configurator's displayed value for one raw EEPROM byte,
    or None if this setting isn't one of the configurator-exposed fields
    this table covers."""
    if raw is None or name not in _CONFIGURATOR_LABELS:
        return None
    raw = int(raw)
    if name == "advance_level":
        return f"{(raw - 10) * 0.9375:.1f}°"
    if name == "pwm_frequency":
        return f"{raw} kHz"
    if name == "variable_pwm":
        return _VARIABLE_PWM_LABELS.get(raw, str(raw))
    if name == "max_ramp":
        return f"{raw * 0.1:.1f} %/ms"
    if name == "minimum_duty_cycle":
        # EEPROM unit * 10 duty counts / 2000 = 0.5% per unit
        # (Src/settings.c: minimum_duty_cycle = eeprom * 10).
        return f"{raw * 0.5:.1f}%"
    if name == "startup_power":
        return f"{raw}%"
    if name == "auto_advance":
        return "On" if raw else "Off"
    return None


def _configurator_settings_rows(
        settings_rows: list[dict]) -> tuple[list[list[str]], set[int]]:
    """([configurator label, configurator value], dim_row_indices) for the
    WINNING ("best") settings, in the standard AM32 tune order - a settings
    table a user can read straight off and type into the AM32 web
    configurator. Raw bytes are already shown in the adjacent settings-diff
    table, so they're intentionally left out here to keep this one
    skimmable.

    When Variable PWM is "By RPM" the configurator itself greys out and
    disables the PWM Frequency field (the firmware picks it dynamically
    from RPM instead) - that row is flagged in ``dim_row_indices`` and
    annotated, so this table doesn't read as "type this value in" for a
    field the real UI won't let you edit."""
    best_by_name = {r.get("setting"): r.get("best") for r in settings_rows}
    by_rpm = best_by_name.get("variable_pwm") == 2

    rows: list[list[str]] = []
    dim: set[int] = set()
    for r in settings_rows:
        name = r.get("setting")
        label = _CONFIGURATOR_LABELS.get(name)
        val = _configurator_value(name, r.get("best"))
        if label is None or val is None:
            continue
        if name == "pwm_frequency" and by_rpm:
            val += " (locked - By RPM sets this)"
            dim.add(len(rows))
        rows.append([label, val])
    return rows, dim


def _final_stage_segment_effs(out_dir: str | Path, manifest: dict,
                              kind: str) -> list[tuple[float, list[float]]]:
    """[(throttle, [eff_gf_per_w per repeat])] sorted by throttle, pooled
    across every steady-point segment seen in the ABBA finals trials of the
    given ``kind`` (``"final_winner"`` or ``"final_default"``)."""
    seg_effs: dict[str, list[float]] = {}
    seg_throttle: dict[str, float] = {}
    for e, metrics in _load_trial_metrics(out_dir, manifest):
        if e.get("kind") != kind or metrics.get("_load_error"):
            continue
        for p in metrics.get("steady_points", []):
            seg, eff = p.get("segment"), p.get("eff_gf_per_w")
            if seg is None or eff is None:
                continue
            seg_effs.setdefault(seg, []).append(eff)
            seg_throttle.setdefault(seg, p.get("throttle"))
    return sorted(((seg_throttle[s], vals) for s, vals in seg_effs.items()
                  if seg_throttle.get(s) is not None),
                 key=lambda t: t[0])


def _peak_efficiency_row(out_dir: str | Path,
                         manifest: dict) -> tuple[float, float] | None:
    """(eff_gf_per_w, throttle) of the most efficient steady-state segment
    for the confirmed winner, medianed across the ABBA final-winner sweeps
    (``final_winner`` trials) - consistent with how every other winner
    metric in this tool is aggregated across repeats. Returns None if no
    final-winner steady-point data is available (e.g. default settings
    were kept, so there's no separate winner sweep)."""
    points = _final_stage_segment_effs(out_dir, manifest, "final_winner")
    if not points:
        return None
    throttle, effs = max(points, key=lambda t: statistics.median(t[1]))
    return statistics.median(effs), throttle


def _pdf_efficiency_curve_page(plt, pdf, out_dir: str | Path, manifest: dict,
                               result: dict) -> None:
    """Full-page efficiency-vs-throttle curve for the confirmed winner
    (median across ABBA repeats, individual repeats shown as faint points),
    with the factory-default sweep overlaid for context. Skipped if there's
    no separate winner sweep to show (e.g. default settings were kept)."""
    winner_pts = _final_stage_segment_effs(out_dir, manifest, "final_winner")
    if not winner_pts:
        return
    default_pts = _final_stage_segment_effs(out_dir, manifest, "final_default")

    fig, ax = plt.subplots(figsize=_PAGE)
    fig.suptitle("Efficiency vs. throttle", fontsize=18, fontweight="bold",
                 x=0.06, ha="left", y=0.97)

    def _plot(points, color, label, marker):
        thr = [t * 100 for t, _ in points]
        med = [statistics.median(vals) for _, vals in points]
        ax.plot(thr, med, marker + "-", color=color, label=label,
               linewidth=2, ms=5, zorder=3)
        for t, vals in points:
            if len(vals) > 1:
                ax.plot([t * 100] * len(vals), vals, marker, color=color,
                       alpha=0.35, ms=4, zorder=2)

    if default_pts:
        _plot(default_pts, _DEFAULT_GRAY, "default (factory)", "s")
    ov = result.get("winner_overrides") or {}
    _plot(winner_pts, _CONFIRMED_GREEN, f"winner ({_fmt_ov(ov)})", "o")

    ax.set_xlabel("throttle %")
    ax.set_ylabel("efficiency (g/W)")
    ax.set_xlim(0, 100)
    ax.set_xticks(range(0, 101, 10))
    ax.set_ylim(bottom=0)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    pdf.savefig(fig)
    plt.close(fig)


def _pdf_cover_page(plt, pdf, out_dir: str | Path, manifest: dict,
                    result: dict, diff_rows: list,
                    settings_rows: list | None = None):
    fig = plt.figure(figsize=_PAGE)
    fig.text(0.06, 0.955, "AM32 Auto-Tune Report", fontsize=22,
             fontweight="bold", va="top")
    fig.text(0.06, 0.925, str(manifest.get("spec_name", "")), fontsize=13,
             color=_DEFAULT_GRAY, va="top")

    # -- metadata block --
    addr = manifest.get("eeprom_address")
    meta_lines = [
        ("mode", manifest.get("mode")),
        ("git_sha", manifest.get("git_sha")),
        ("eeprom_address",
         f"0x{addr:08x}" if isinstance(addr, int) else addr),
        ("battery_cells", manifest.get("battery_cells")),
        ("jitter reference (default)",
         f"{manifest.get('jitter_reference')} %"),
        ("pack swaps", len(manifest.get("pack_events", []))),
        ("trials run", len(manifest.get("trials", []))),
    ]
    y = 0.885
    for k, v in meta_lines:
        fig.text(0.06, y, f"{k}", fontsize=10, color=_DEFAULT_GRAY, va="top")
        fig.text(0.42, y, _fmt(v), fontsize=10, va="top")
        y -= 0.024

    # -- verdict banner --
    confirmed = bool(result.get("confirmed"))
    banner_color = _CONFIRMED_GREEN if confirmed else _DEFAULT_GRAY
    verdict = ("WINNER CONFIRMED" if confirmed
               else "DEFAULT SETTINGS KEPT")
    ax = fig.add_axes([0.06, 0.60, 0.88, 0.09])
    ax.axis("off")
    ax.add_patch(plt.Rectangle((0, 0), 1, 1, transform=ax.transAxes,
                               facecolor=banner_color, alpha=0.14,
                               edgecolor=banner_color, linewidth=1.5))
    ax.text(0.5, 0.5, verdict, transform=ax.transAxes, ha="center",
            va="center", fontsize=20, fontweight="bold", color=banner_color)

    ov = result.get("winner_overrides") or {}
    detail = [
        f"winner overrides: {_fmt_ov(ov)}",
        (f"median paired delta: {_fmt(result.get('median_paired_delta'))} g/W "
         f"over {len(result.get('paired_deltas') or [])} ABBA pairs"),
        f"winner constraint failures: {result.get('winner_constraint_failures', 0)}",
    ]
    st = result.get("startup")
    if st is not None:
        detail.append(f"startup: {st.get('failed')}/{st.get('cycles')} failed")
    peak = _peak_efficiency_row(out_dir, manifest) if confirmed else None
    if peak is not None:
        eff, throttle = peak
        detail.append(f"peak efficiency: {eff:.3f} g/W at "
                      f"{throttle * 100:.0f}% throttle")
    # Fixed line_h (0.026) at up to 4 lines matches the fixed 0.46 y-position
    # of the "Settings diff" header below; compress spacing for any extra
    # lines instead of overflowing into that header.
    y = 0.565
    line_h = min(0.026, (0.565 - 0.484) / max(len(detail) - 1, 1))
    for line in detail:
        fig.text(0.06, y, line, fontsize=11, va="top")
        y -= line_h

    # -- settings diff table (left) + AM32 configurator translation (right) --
    fig.text(0.06, 0.46, "Settings diff (default → best)", fontsize=13,
             fontweight="bold", va="top")
    if diff_rows:
        rows = [[name, _fmt(a), _fmt(b)] for name, a, b in diff_rows]
    else:
        rows = [["(no change — default settings won)", "", ""]]
    _place_table(fig.add_axes([0.06, 0.29, 0.42, 0.15]), rows,
                 ["setting", "default", "best"], [0.48, 0.26, 0.26],
                 line_h=0.10)

    fig.text(0.52, 0.46, "AM32 configurator settings (winner)", fontsize=13,
             fontweight="bold", va="top")
    config_rows, config_dim_rows = _configurator_settings_rows(settings_rows or [])
    _place_table(fig.add_axes([0.52, 0.29, 0.42, 0.15]), config_rows,
                 ["setting", "value"], [0.55, 0.45], line_h=0.10,
                 dim_rows=config_dim_rows)

    # -- stage winners (was its own near-empty page) --
    fig.text(0.06, 0.25, "Stage winners", fontsize=13,
             fontweight="bold", va="top")
    stage_rows = [[name, _wrap(_fmt_ov(s.get("winner")), 58),
                   _fmt(s.get("winner_score"))]
                  for name, s in manifest.get("stages", {}).items()]
    _place_table(fig.add_axes([0.06, 0.05, 0.88, 0.18]), stage_rows,
                 ["stage", "winner", "score (g/W, norm)"],
                 [0.15, 0.62, 0.23], line_h=0.085)
    pdf.savefig(fig)
    plt.close(fig)


def _pdf_progress_page(plt, pdf, manifest: dict, result: dict):
    from matplotlib.ticker import MaxNLocator
    from matplotlib.transforms import blended_transform_factory

    trials = [t for t in manifest.get("trials", []) if not t.get("discarded")]
    fig, (ax_prog, ax_abba) = plt.subplots(2, 1, figsize=_PAGE)
    fig.suptitle("Tuning progress", fontsize=18, fontweight="bold", x=0.06,
                 ha="left", y=0.97)

    # scored trials: raw + normalized g/W vs trial index; anchors (incumbent
    # re-runs) marked distinctly so drift is readable at a glance
    idx_raw = [(t["index"], t["score_raw"]) for t in trials
               if t.get("score_raw") is not None]
    idx_norm = [(t["index"], t["score_norm"]) for t in trials
                if t.get("score_norm") is not None]
    anchors = [(t["index"], t["score_raw"]) for t in trials
               if t.get("kind") == "anchor"
               and t.get("score_raw") is not None]
    dq = [t["index"] for t in trials if t.get("disqualified")]

    if idx_raw:
        xs, ys = zip(*idx_raw)
        ax_prog.plot(xs, ys, "o-", color="#0969da", label="raw g/W", ms=4)
    if idx_norm:
        xs, ys = zip(*idx_norm)
        ax_prog.plot(xs, ys, "s--", color="#bf8700",
                     label="drift-normalized g/W", ms=4)
    if anchors:
        xs, ys = zip(*anchors)
        ax_prog.plot(xs, ys, "o", mfc="white", mec="#0969da", ms=8,
                     ls="none", label="anchor (incumbent re-run)")
    if dq:
        # park DQ markers in their own band under the data instead of on
        # top of the minimum score (they have no score of their own)
        ys = [y for _, y in idx_raw + idx_norm]
        lo, hi = (min(ys), max(ys)) if ys else (0.0, 1.0)
        band = lo - 0.08 * (hi - lo or 1.0)
        ax_prog.plot(dq, [band] * len(dq), "x", color="#cf222e", ms=8,
                     mew=2, ls="none", label="disqualified (no score)")

    # stage bands: alternating background + label per contiguous stage
    trans = blended_transform_factory(ax_prog.transData, ax_prog.transAxes)
    spans: list[tuple[str, int, int]] = []
    for t in sorted(trials, key=lambda t: t["index"]):
        if spans and spans[-1][0] == t["stage"]:
            spans[-1] = (t["stage"], spans[-1][1], t["index"])
        else:
            spans.append((t["stage"], t["index"], t["index"]))
    for i, (name, a, b) in enumerate(spans):
        if i % 2:
            ax_prog.axvspan(a - 0.5, b + 0.5, color="#afb8c1", alpha=0.15,
                            lw=0)
        ax_prog.text((a + b) / 2.0, 1.015, name, transform=trans,
                     ha="center", va="bottom", fontsize=8,
                     color=_DEFAULT_GRAY)

    ax_prog.set(xlabel="trial index", ylabel="objective (g/W)")
    ax_prog.set_title("Objective per trial", pad=22)
    ax_prog.xaxis.set_major_locator(MaxNLocator(integer=True))
    if idx_raw or idx_norm or dq:
        ax_prog.legend(loc="best", fontsize=8)
    ax_prog.grid(True, alpha=0.3)

    # ABBA paired deltas (winner - default), median line
    deltas = result.get("paired_deltas") or []
    if deltas:
        colors = ["#1a7f37" if d > 0 else "#cf222e" for d in deltas]
        ax_abba.bar(range(1, len(deltas) + 1), deltas, color=colors,
                    alpha=0.8, width=0.5)
        med = result.get("median_paired_delta")
        if med is not None:
            ax_abba.axhline(med, ls="--", color="#0969da",
                            label=f"median {med:g} g/W")
        ax_abba.axhline(0, color="#57606a", lw=0.8)
        ax_abba.set_xticks(list(range(1, len(deltas) + 1)))
        ax_abba.legend(loc="best", fontsize=8)
    else:
        ax_abba.text(0.5, 0.5, "no ABBA paired deltas", ha="center",
                     va="center", transform=ax_abba.transAxes,
                     color=_DEFAULT_GRAY)
    ax_abba.set(xlabel="ABBA pair", ylabel="winner − default (g/W)",
                title="Finals: interleaved paired deltas")
    ax_abba.grid(True, alpha=0.3)

    fig.tight_layout(rect=(0, 0, 1, 0.95))
    pdf.savefig(fig)
    plt.close(fig)


def _pdf_settings_pages(plt, pdf, settings_rows: list) -> None:
    if not settings_rows:
        return
    rows = [[_fmt(r.get("setting")), _fmt(r.get("offset")),
             _fmt(r.get("default")), _fmt(r.get("best")),
             "yes" if r.get("changed") else ""]
            for r in settings_rows]
    for i, chunk in enumerate(_paginate_rows(rows, 34)):
        fig = plt.figure(figsize=_PAGE_LS)
        title = "Full settings" if i == 0 else f"Full settings (cont. {i + 1})"
        fig.text(0.04, 0.94, title, fontsize=18, fontweight="bold", va="top")
        _place_table(fig.add_axes([0.04, 0.05, 0.92, 0.80]), chunk,
                     ["setting", "offset", "default", "best", "changed"],
                     [0.42, 0.10, 0.16, 0.16, 0.12], line_h=0.026)
        pdf.savefig(fig)
        plt.close(fig)


def _pdf_settings_impact_pages(plt, pdf, manifest: dict,
                               settings_rows: list) -> None:
    rows = _settings_impact_rows(manifest, settings_rows)
    if not rows:
        return
    table_rows = [[
        _fmt(r["stage"]), _wrap(r["setting"], 24), _fmt(r["value"]),
        _fmt(r["runs"]), _fmt(r["disqualified"]),
        f"{_fmt(r['median_raw'])}/{_fmt(r['median_norm'])}",
        _fmt_delta(r["delta_norm"]), _wrap(r["trials"], 38),
    ] for r in rows]
    for i, chunk in enumerate(_paginate_rows(table_rows, 32)):
        fig = plt.figure(figsize=_PAGE_LS)
        title = ("Settings performance impact" if i == 0
                 else f"Settings performance impact (cont. {i + 1})")
        fig.text(0.04, 0.94, title, fontsize=18, fontweight="bold", va="top")
        _place_table(fig.add_axes([0.04, 0.05, 0.92, 0.80]), chunk,
                     ["stage", "setting", "value", "runs", "DQ",
                      "median raw/norm", "delta", "trials"],
                     [0.10, 0.20, 0.09, 0.06, 0.05, 0.16, 0.10, 0.24],
                     fontsize=7.4, line_h=0.026)
        pdf.savefig(fig)
        plt.close(fig)


def _fmt_ov(ov) -> str:
    """Compact one-line overrides: {'a': 1, 'b': 2} -> 'a=1, b=2'."""
    if not ov:
        return "default"
    if isinstance(ov, dict):
        return ", ".join(f"{k}={v}" for k, v in ov.items())
    return str(ov)


_TRIAL_COLS = ["#", "stage", "kind", "overrides", "raw g/W", "norm g/W",
               "disqualified"]
_TRIAL_WIDTHS = [0.04, 0.09, 0.11, 0.40, 0.075, 0.075, 0.21]


def _fmt_raw(v) -> str:
    if isinstance(v, (dict, list)):
        return json.dumps(v, sort_keys=True)
    return _fmt(v)


def _md_cell(v) -> str:
    return _fmt_raw(v).replace("|", "\\|").replace("\n", "<br>")


def _fmt_delta(v) -> str:
    return "-" if v is None else f"{v:+.3f}"


def _median(values: list) -> float | None:
    values = [v for v in values if v is not None]
    return statistics.median(values) if values else None


def _score_norm(e: dict):
    return (e.get("score_norm") if e.get("score_norm") is not None
            else e.get("score_raw"))


def _settings_impact_rows(manifest: dict, settings_rows: list) -> list[dict]:
    defaults = {r.get("setting"): r.get("default") for r in settings_rows
                if r.get("setting") is not None}
    if not defaults:
        return []
    setting_order = {r.get("setting"): i for i, r in enumerate(settings_rows)}
    trials = [t for t in manifest.get("trials", []) if not t.get("discarded")]

    stage_order: list[str] = []
    stage_settings: dict[str, set] = {}
    for e in trials:
        stage = e.get("stage")
        if stage is None:
            continue
        if stage not in stage_order:
            stage_order.append(stage)
        ov = e.get("overrides") if isinstance(e.get("overrides"), dict) else {}
        kind = str(e.get("kind") or "")
        if kind == "trial" or kind.startswith("final_"):
            stage_settings.setdefault(stage, set()).update(ov)

    groups: dict[tuple[str, str, object], list[dict]] = {}
    for e in trials:
        stage = e.get("stage")
        if stage not in stage_settings or e.get("kind") == "final_startup":
            continue
        ov = e.get("overrides") if isinstance(e.get("overrides"), dict) else {}
        for setting in stage_settings[stage]:
            if setting not in ov and setting not in defaults:
                continue
            value = ov.get(setting, defaults.get(setting))
            groups.setdefault((stage, setting, value), []).append(e)

    out = []
    for stage in stage_order:
        settings = sorted(stage_settings.get(stage, set()),
                          key=lambda s: setting_order.get(s, 9999))
        for setting in settings:
            value_entries = {value: entries for (st, name, value), entries
                             in groups.items()
                             if st == stage and name == setting}
            if len(value_entries) < 2:
                continue
            default_value = defaults.get(setting)
            default_norm = _median([_score_norm(e)
                                    for e in value_entries.get(default_value, [])])
            values = sorted(value_entries,
                            key=lambda v: (v != default_value,
                                           v if isinstance(v, (int, float)) else 0,
                                           str(v)))
            for value in values:
                entries = value_entries[value]
                median_norm = _median([_score_norm(e) for e in entries])
                delta = (None if median_norm is None or default_norm is None
                         else median_norm - default_norm)
                out.append({
                    "stage": stage,
                    "setting": setting,
                    "value": value,
                    "runs": len(entries),
                    "disqualified": sum(1 for e in entries
                                         if e.get("disqualified")),
                    "median_raw": _median([e.get("score_raw") for e in entries]),
                    "median_norm": median_norm,
                    "delta_norm": delta,
                    "trials": ",".join(str(e.get("index")) for e in entries),
                })
    return out


def render_tune_settings_impact_markdown(manifest: dict,
                                         settings_rows: list) -> str:
    rows = _settings_impact_rows(manifest, settings_rows)
    if not rows:
        return ""
    out = ["## Settings performance impact\n",
           "| stage | setting | value | runs | DQ | median raw g/W | "
           "median norm g/W | delta vs default norm g/W | trials |",
           "|---|---|---:|---:|---:|---:|---:|---:|---|"]
    for r in rows:
        out.append(f"| {_md_cell(r['stage'])} | `{_md_cell(r['setting'])}` | "
                   f"{_md_cell(r['value'])} | {r['runs']} | "
                   f"{r['disqualified']} | {_md_cell(r['median_raw'])} | "
                   f"{_md_cell(r['median_norm'])} | "
                   f"{_fmt_delta(r['delta_norm'])} | {_md_cell(r['trials'])} |")
    out.append("")
    return "\n".join(out)


def _load_trial_metrics(out_dir: str | Path,
                        manifest: dict) -> list[tuple[dict, dict]]:
    out = []
    root = Path(out_dir)
    for e in manifest.get("trials", []):
        path = root / e.get("dir", "") / "metrics.json"
        try:
            metrics = json.loads(path.read_text())
        except Exception as exc:
            metrics = {"_load_error": f"{path}: {exc}"}
        out.append((e, metrics))
    return out


def _paginate_rows(rows: list[list[str]],
                   line_budget: int) -> list[list[list[str]]]:
    pages: list[list[list[str]]] = [[]]
    lines = 0
    for row in rows:
        n = max(cell.count("\n") + 1 for cell in row)
        if pages[-1] and lines + n > line_budget:
            pages.append([])
            lines = 0
        pages[-1].append(row)
        lines += n
    return pages


def _iter_run_metrics(metrics: dict):
    for name, value in metrics.get("summary", {}).items():
        yield name, value
    for section in ("demag", "startup"):
        block = metrics.get(section)
        if not isinstance(block, dict):
            continue
        for name, value in block.items():
            yield f"{section}.{name}", value


def render_tune_raw_markdown(out_dir: str | Path, manifest: dict) -> str:
    """Long-form high-level raw metrics from every tune trial."""
    data = _load_trial_metrics(out_dir, manifest)
    out: list[str] = []
    out.append("## Run summary raw data\n")
    out.append("| # | stage | kind | profile | metric | value |")
    out.append("|---|---|---|---|---|---|")
    for e, metrics in data:
        if metrics.get("_load_error"):
            out.append(f"| {e.get('index')} | {_md_cell(e.get('stage'))} | "
                       f"{_md_cell(e.get('kind'))} | {_md_cell(e.get('profile'))} | "
                       f"metrics.json | {_md_cell(metrics['_load_error'])} |")
            continue
        for name, value in _iter_run_metrics(metrics):
            out.append(f"| {e.get('index')} | {_md_cell(e.get('stage'))} | "
                       f"{_md_cell(e.get('kind'))} | {_md_cell(e.get('profile'))} | "
                       f"{_md_cell(name)} | {_md_cell(value)} |")
    out.append("")

    out.append("## Steady-point raw data\n")
    out.append("| # | stage | kind | profile | segment | metric | value |")
    out.append("|---|---|---|---|---|---|---|")
    for e, metrics in data:
        if metrics.get("_load_error"):
            continue
        for point in metrics.get("steady_points", []):
            segment = point.get("segment")
            for name, value in point.items():
                if name == "segment":
                    continue
                out.append(f"| {e.get('index')} | {_md_cell(e.get('stage'))} | "
                           f"{_md_cell(e.get('kind'))} | "
                           f"{_md_cell(e.get('profile'))} | "
                           f"{_md_cell(segment)} | {_md_cell(name)} | "
                           f"{_md_cell(value)} |")
    out.append("")
    return "\n".join(out)


# zc_phase_hist is a per-bin commutation histogram (a list of ~20-30 counts,
# not a scalar) - it doesn't fit a table cell and is excluded from the raw
# data pages; read metrics.json directly for it.
_STEADY_ID_FIELDS = {"segment", "throttle"}
_STEADY_SKIP_FIELDS = _STEADY_ID_FIELDS | {"zc_phase_hist"}
_RAW_GROUP_SIZE = 6


def _chunked(seq: list, n: int) -> list[list]:
    return [seq[i:i + n] for i in range(0, len(seq), n)]


def _wide_metric_pages(plt, pdf, title: str, id_cols: list[str],
                       id_widths: list[float], entries: list[tuple[list, dict]],
                       metric_names: list[str], group_size: int) -> None:
    """One set of pages per ``group_size`` metric columns, a row per entry
    (trial, or trial+segment) - unlike a melted one-row-per-metric layout,
    this doesn't repeat the identifying columns for every metric and scales
    to far fewer pages for the same data."""
    metric_width = (0.92 - sum(id_widths)) / max(min(group_size,
                                                      len(metric_names)), 1)
    for gi, names in enumerate(_chunked(metric_names, group_size)):
        cols = list(id_cols) + [_wrap(n, 14) for n in names]
        widths = list(id_widths) + [metric_width] * len(names)
        rows = [list(ids) + [_wrap(_fmt_raw(vals.get(n)), 14) for n in names]
                for ids, vals in entries]
        for pi, chunk in enumerate(_paginate_rows(rows, 26)):
            fig = plt.figure(figsize=_PAGE_LS)
            page_title = title
            if len(metric_names) > group_size:
                page_title += f" ({gi + 1}/{-(-len(metric_names) // group_size)})"
            if pi:
                page_title += f" cont. {pi + 1}"
            fig.text(0.04, 0.94, page_title, fontsize=16,
                     fontweight="bold", va="top")
            _place_table(fig.add_axes([0.04, 0.05, 0.92, 0.80]), chunk,
                         cols, widths, fontsize=7.2, line_h=0.026)
            pdf.savefig(fig)
            plt.close(fig)


def _pdf_raw_data_pages(plt, pdf, out_dir: str | Path, manifest: dict) -> None:
    data = _load_trial_metrics(out_dir, manifest)

    error_rows = [[_fmt(e.get("index")),
                   _wrap(f"{e.get('stage')}/{e.get('kind')}", 20),
                   _fmt(e.get("profile")), _wrap(m["_load_error"], 70)]
                  for e, m in data if m.get("_load_error")]
    if error_rows:
        for pi, chunk in enumerate(_paginate_rows(error_rows, 26)):
            fig = plt.figure(figsize=_PAGE_LS)
            title = "Run summary raw data - load errors" + (
                f" cont. {pi + 1}" if pi else "")
            fig.text(0.04, 0.94, title, fontsize=16, fontweight="bold", va="top")
            _place_table(fig.add_axes([0.04, 0.05, 0.92, 0.80]), chunk,
                         ["#", "stage/kind", "profile", "error"],
                         [0.05, 0.20, 0.12, 0.63], fontsize=7.2, line_h=0.026)
            pdf.savefig(fig)
            plt.close(fig)

    summary_names: list[str] = []
    summary_entries: list[tuple[list, dict]] = []
    for e, metrics in data:
        if metrics.get("_load_error"):
            continue
        vals = dict(_iter_run_metrics(metrics))
        summary_names.extend(k for k in vals if k not in summary_names)
        ids = [_fmt(e.get("index")),
               _wrap(f"{e.get('stage')}/{e.get('kind')}", 18),
               _fmt(e.get("profile"))]
        summary_entries.append((ids, vals))
    if summary_entries:
        _wide_metric_pages(plt, pdf, "Run summary raw data",
                           ["#", "stage/kind", "profile"], [0.04, 0.16, 0.08],
                           summary_entries, summary_names, _RAW_GROUP_SIZE)

    steady_names: list[str] = []
    steady_entries: list[tuple[list, dict]] = []
    for e, metrics in data:
        if metrics.get("_load_error"):
            continue
        stage_kind = f"{e.get('stage')}/{e.get('kind')}"
        for point in metrics.get("steady_points", []):
            vals = {k: v for k, v in point.items()
                    if k not in _STEADY_SKIP_FIELDS}
            steady_names.extend(k for k in vals if k not in steady_names)
            ids = [_fmt(e.get("index")), _wrap(stage_kind, 16),
                   _fmt(point.get("segment")), _fmt(point.get("throttle"))]
            steady_entries.append((ids, vals))
    if steady_entries:
        _wide_metric_pages(plt, pdf, "Steady-point raw data",
                           ["#", "stage/kind", "segment", "throttle"],
                           [0.04, 0.14, 0.07, 0.07], steady_entries,
                           steady_names, _RAW_GROUP_SIZE)


def _pdf_tables_pages(plt, pdf, manifest: dict):
    # trials table: landscape + explicit widths + wrapped text (a portrait
    # auto-width table clipped both edges on real sessions), paginated on a
    # LINE budget so wrapped disqualification reasons don't overflow a page
    trial_rows = []
    for e in manifest.get("trials", []):
        dq = "; ".join(e["disqualified"]) if e.get("disqualified") else ""
        if e.get("discarded"):
            dq = (dq + " " if dq else "") + "(discarded: pack swap)"
        trial_rows.append([
            _fmt(e.get("index")), _fmt(e.get("stage")), _fmt(e.get("kind")),
            _wrap(_fmt_ov(e.get("overrides")), 62), _fmt(e.get("score_raw")),
            _fmt(e.get("score_norm")), _wrap(dq, 38)])

    for i, chunk in enumerate(_paginate_rows(trial_rows, 30)):
        fig = plt.figure(figsize=_PAGE_LS)
        title = "Trials" if i == 0 else f"Trials (cont. {i + 1})"
        fig.text(0.04, 0.94, title, fontsize=18, fontweight="bold", va="top")
        _place_table(fig.add_axes([0.04, 0.05, 0.92, 0.80]), chunk,
                     _TRIAL_COLS, _TRIAL_WIDTHS, line_h=0.026)
        pdf.savefig(fig)
        plt.close(fig)


def _write_plots(run_dir: Path, metrics: dict) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    pts = metrics.get("steady_points", [])
    if not pts:
        return
    thr = [p["throttle"] * 100 for p in pts]

    fig, axes = plt.subplots(2, 2, figsize=(10, 7))
    axes[0, 0].plot(thr, [p["thrust_gf"] for p in pts], "o-")
    axes[0, 0].set(title="Thrust", xlabel="throttle %", ylabel="gf")
    axes[0, 1].plot(thr, [p["eff_gf_per_w"] for p in pts], "o-", color="green")
    axes[0, 1].set(title="Efficiency", xlabel="throttle %", ylabel="g/W")
    axes[1, 0].plot(thr, [p["cpu_load_pct"] for p in pts], "o-", color="red")
    axes[1, 0].set(title="CPU load", xlabel="throttle %", ylabel="%")
    axes[1, 1].plot(thr, [p["ctrl_exec_us_max"] for p in pts], "o-", color="purple")
    axes[1, 1].axhline(50, ls="--", color="gray", label="20kHz budget")
    axes[1, 1].set(title="Control-loop exec", xlabel="throttle %", ylabel="us")
    axes[1, 1].legend()
    fig.tight_layout()
    fig.savefig(run_dir / "summary.png", dpi=110)
    plt.close(fig)
