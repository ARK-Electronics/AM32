"""Auto-tune PDF report rendering (report.write_tune_pdf)."""
import json
import sys

import pytest

from hwci import report

# The whole PDF path is a best-effort matplotlib feature; skip when it's not
# installed (the tune itself never depends on it).
mpl = pytest.importorskip("matplotlib")


def _manifest(**over):
    m = {
        "spec_name": "unit", "mode": "sim", "git_sha": "deadbee",
        "eeprom_address": 0x08007C00, "battery_cells": 6,
        "jitter_reference": 0.8, "pack_events": [],
        "stages": {"advance": {"winner": {"advance_level": 30},
                               "winner_score": 6.30}},
        "trials": [
            {"index": 0, "stage": "baseline", "kind": "baseline",
             "overrides": {}, "score_raw": 5.9, "score_norm": 5.9,
             "disqualified": None, "discarded": False},
            {"index": 1, "stage": "advance", "kind": "trial",
             "overrides": {"advance_level": 30}, "score_raw": 6.3,
             "score_norm": 6.3, "disqualified": None, "discarded": False},
            {"index": 2, "stage": "advance", "kind": "trial",
             "overrides": {"advance_level": 10}, "score_raw": None,
             "score_norm": None, "disqualified": ["demag"],
             "discarded": False},
        ],
    }
    m.update(over)
    return m


def _result(**over):
    r = {"confirmed": True, "winner_overrides": {"advance_level": 30},
         "median_paired_delta": 0.2, "paired_deltas": [0.1, 0.3, -0.05],
         "winner_constraint_failures": 0, "startup": {"failed": 0, "cycles": 5}}
    r.update(over)
    return r


def _is_pdf(path) -> bool:
    return path is not None and path.read_bytes()[:5] == b"%PDF-"


def test_write_tune_pdf_produces_valid_pdf(tmp_path):
    p = report.write_tune_pdf(tmp_path, _manifest(), _result(),
                              [("advance_level", 26, 30)])
    assert p == tmp_path / "tune_report.pdf"
    assert _is_pdf(p)


def test_write_tune_pdf_default_kept_no_deltas(tmp_path):
    # not confirmed, empty diff, no ABBA deltas, no stages/trials
    m = _manifest(stages={}, trials=[])
    r = _result(confirmed=False, winner_overrides={},
                median_paired_delta=None, paired_deltas=[], startup=None)
    p = report.write_tune_pdf(tmp_path, m, r, [])
    assert _is_pdf(p)


def test_write_tune_pdf_paginates_many_trials(tmp_path):
    trials = [{"index": i, "stage": "advance", "kind": "trial",
               "overrides": {"advance_level": i}, "score_raw": 6.0 + i * 0.01,
               "score_norm": 6.0 + i * 0.01, "disqualified": None,
               "discarded": False} for i in range(60)]
    p = report.write_tune_pdf(tmp_path, _manifest(trials=trials), _result(), [])
    assert _is_pdf(p)


def test_render_tune_raw_markdown_includes_trial_metrics(tmp_path):
    trial_dir = tmp_path / "trials" / "T000-baseline"
    trial_dir.mkdir(parents=True)
    (trial_dir / "metrics.json").write_text(json.dumps({
        "summary": {"max_current_a": 12.3, "n_samples": 42},
        "demag": {"event_count": 1},
        "startup": {"attempts": 2, "failures": 0},
        "steady_points": [{"segment": "s50", "rpm": 1000.0,
                           "eff_gf_per_w": 4.2}],
    }))
    manifest = _manifest(trials=[{
        "index": 0, "stage": "baseline", "kind": "baseline",
        "profile": "probe", "overrides": {}, "score_raw": 4.2,
        "score_norm": 4.2, "disqualified": None, "discarded": False,
        "dir": "trials/T000-baseline",
    }])

    md = report.render_tune_raw_markdown(tmp_path, manifest)

    assert "## Run summary raw data" in md
    assert "max_current_a" in md
    assert "demag.event_count" in md
    assert "startup.attempts" in md
    assert "## Steady-point raw data" in md
    assert "eff_gf_per_w" in md


def test_render_tune_settings_impact_markdown_groups_by_value():
    manifest = _manifest(trials=[
        {"index": 0, "stage": "advance", "kind": "anchor",
         "overrides": {}, "score_raw": 5.0, "score_norm": 5.0,
         "disqualified": None, "discarded": False},
        {"index": 1, "stage": "advance", "kind": "trial",
         "overrides": {"advance_level": 30}, "score_raw": 6.3,
         "score_norm": 6.3, "disqualified": None, "discarded": False},
        {"index": 2, "stage": "advance", "kind": "trial",
         "overrides": {"advance_level": 34}, "score_raw": 6.1,
         "score_norm": 6.2, "disqualified": None, "discarded": False},
        {"index": 3, "stage": "advance", "kind": "trial",
         "overrides": {"advance_level": 10}, "score_raw": None,
         "score_norm": None, "disqualified": ["demag"],
         "discarded": False},
    ])
    settings_rows = [{"setting": "advance_level", "offset": 23,
                      "default": 26, "best": 30, "changed": True}]

    md = report.render_tune_settings_impact_markdown(manifest, settings_rows)

    assert "## Settings performance impact" in md
    assert "advance_level" in md
    assert "+1.300" in md
    assert "| advance | `advance_level` | 10 | 1 | 1 | - | - | - | 3 |" in md


def test_write_tune_pdf_skips_gracefully_without_matplotlib(tmp_path, monkeypatch):
    # Force `import matplotlib` to fail; the tune must not blow up.
    monkeypatch.setitem(sys.modules, "matplotlib", None)
    logs = []
    p = report.write_tune_pdf(tmp_path, _manifest(), _result(), [],
                              log=logs.append)
    assert p is None
    assert not (tmp_path / "tune_report.pdf").exists()
    assert any("matplotlib" in m for m in logs)


def test_fmt_ov_compact():
    assert report._fmt_ov({}) == "default"
    assert report._fmt_ov({"advance_level": 30}) == "advance_level=30"
    assert report._fmt_ov({"a": 1, "b": 2}) == "a=1, b=2"


def test_tune_run_writes_pdf_report(tmp_path):
    # end-to-end: a real sim tune emits tune_report.pdf alongside report.md
    from test_tuner import make_backend, run_tune, small_spec

    _, result = run_tune(tmp_path, small_spec(), make_backend(advance_optimum=33.0))
    pdf = tmp_path / "tune" / "tune_report.pdf"
    assert (tmp_path / "tune" / "report.md").exists()
    assert _is_pdf(pdf)


# --------------------------------------------------------------------------
# AM32 configurator settings translation
# --------------------------------------------------------------------------
def test_configurator_value_advance_level_degrees():
    # wiki.am32.ca's EEPROM doc: old-format raw 2 = 15 degrees; main.c
    # converts that to new-format raw 26 ((2<<3)+10) - so 26 must round-trip
    # to 15.0 degrees under the new-format formula.
    assert report._configurator_value("advance_level", 26) == "15.0°"
    assert report._configurator_value("advance_level", 10) == "0.0°"   # floor
    assert report._configurator_value("advance_level", 42) == "30.0°"  # ceiling
    assert report._configurator_value("advance_level", 20) == "9.4°"


def test_configurator_value_pwm_frequency_is_direct_khz():
    assert report._configurator_value("pwm_frequency", 48) == "48 kHz"


def test_configurator_value_variable_pwm_labels():
    assert report._configurator_value("variable_pwm", 0) == "Fixed"
    assert report._configurator_value("variable_pwm", 1) == "Variable (Low/High)"
    assert report._configurator_value("variable_pwm", 2) == "By RPM"


def test_configurator_value_max_ramp_is_direct_percent_per_ms():
    # Inc/eeprom.h: max_ramp raw units are 0.1%/ms steps.
    assert report._configurator_value("max_ramp", 40) == "4.0 %/ms"
    assert report._configurator_value("max_ramp", 1) == "0.1 %/ms"


def test_configurator_value_startup_power_is_direct_percent():
    assert report._configurator_value("startup_power", 100) == "100%"


def test_configurator_value_auto_advance_on_off():
    assert report._configurator_value("auto_advance", 0) == "Off"
    assert report._configurator_value("auto_advance", 1) == "On"


def test_configurator_value_unknown_field_and_none_are_excluded():
    assert report._configurator_value("startup_power", None) is None
    assert report._configurator_value("not_a_real_field", 5) is None


def _rows_with_variable_pwm(best_variable_pwm):
    return [
        {"setting": "max_ramp", "offset": 5, "default": 15, "best": 40},
        {"setting": "variable_pwm", "offset": 21, "default": 1,
         "best": best_variable_pwm},
        {"setting": "advance_level", "offset": 23, "default": 26, "best": 20},
        {"setting": "pwm_frequency", "offset": 24, "default": 24, "best": 48},
        {"setting": "startup_power", "offset": 25, "default": 100, "best": 100},
        {"setting": "auto_advance", "offset": 47, "default": 0, "best": 0},
    ]


def test_configurator_settings_rows_uses_best_in_standard_order():
    rows, dim = report._configurator_settings_rows(
        _rows_with_variable_pwm(1))   # Variable (Low/High), not By RPM
    assert rows == [
        ["Ramp Speed", "4.0 %/ms"],
        ["Variable PWM", "Variable (Low/High)"],
        ["Motor Timing", "9.4°"],
        ["PWM Frequency", "48 kHz"],
        ["Startup Power", "100%"],
        ["Auto Timing Advance", "Off"],
    ]
    assert dim == set()   # PWM Frequency is a live, settable field here


def test_configurator_settings_rows_dims_pwm_frequency_when_by_rpm():
    # The configurator itself greys out/disables PWM Frequency once
    # Variable PWM is set to "By RPM" - the firmware picks it dynamically.
    rows, dim = report._configurator_settings_rows(
        _rows_with_variable_pwm(2))
    pwm_freq_index = [r[0] for r in rows].index("PWM Frequency")
    assert rows[pwm_freq_index] == [
        "PWM Frequency", "48 kHz (locked - By RPM sets this)"]
    assert dim == {pwm_freq_index}


def test_configurator_settings_rows_skips_unlisted_fields():
    settings_rows = [{"setting": "some_future_field", "best": 5}]
    assert report._configurator_settings_rows(settings_rows) == ([], set())


def test_peak_efficiency_row_picks_best_segment_median_across_repeats(tmp_path):
    for i, effs in enumerate([{"t10": 4.0, "t30": 4.3}, {"t10": 4.2, "t30": 4.5}]):
        d = tmp_path / "trials" / f"T{i:03d}-final-winner"
        d.mkdir(parents=True)
        (d / "metrics.json").write_text(json.dumps({
            "summary": {}, "demag": {}, "startup": {},
            "steady_points": [
                {"segment": "t10", "throttle": 0.1, "eff_gf_per_w": effs["t10"]},
                {"segment": "t30", "throttle": 0.3, "eff_gf_per_w": effs["t30"]},
            ],
        }))
    manifest = _manifest(trials=[
        {"index": 0, "stage": "finals", "kind": "final_winner",
         "overrides": {}, "dir": "trials/T000-final-winner"},
        {"index": 1, "stage": "finals", "kind": "final_default",
         "overrides": {}, "dir": "trials/T000-final-winner"},
        {"index": 2, "stage": "finals", "kind": "final_winner",
         "overrides": {}, "dir": "trials/T001-final-winner"},
    ])

    assert report._peak_efficiency_row(tmp_path, manifest) == (4.4, 0.3)


def test_peak_efficiency_row_returns_none_without_final_winner_data():
    manifest = _manifest(trials=[
        {"index": 0, "stage": "baseline", "kind": "baseline", "overrides": {}},
    ])
    assert report._peak_efficiency_row("/nonexistent", manifest) is None


class _PageRecorder:
    def __init__(self):
        self.pages = []

    def savefig(self, fig):
        self.pages.append(fig)


def _pyplot():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def test_pdf_efficiency_curve_page_skipped_without_winner_sweep(tmp_path):
    plt = _pyplot()
    manifest = _manifest(trials=[
        {"index": 0, "stage": "baseline", "kind": "baseline", "overrides": {}},
    ])

    rec = _PageRecorder()
    report._pdf_efficiency_curve_page(plt, rec, tmp_path, manifest, _result())
    plt.close("all")

    assert rec.pages == []


def test_pdf_efficiency_curve_page_plots_winner_and_default(tmp_path):
    plt = _pyplot()
    trials = []
    for idx, kind, eff in [(0, "final_winner", 4.0),
                           (1, "final_default", 3.0),
                           (2, "final_winner", 4.4)]:
        d = tmp_path / "trials" / f"T{idx:03d}"
        d.mkdir(parents=True)
        (d / "metrics.json").write_text(json.dumps({
            "summary": {}, "demag": {}, "startup": {},
            "steady_points": [{"segment": "t50", "throttle": 0.5,
                               "eff_gf_per_w": eff}],
        }))
        trials.append({"index": idx, "stage": "finals", "kind": kind,
                       "overrides": {}, "dir": f"trials/T{idx:03d}"})
    manifest = _manifest(trials=trials)

    rec = _PageRecorder()
    report._pdf_efficiency_curve_page(
        plt, rec, tmp_path, manifest,
        _result(winner_overrides={"advance_level": 20}))
    plt.close("all")

    assert len(rec.pages) == 1
    ax = rec.pages[0].axes[0]
    _, labels = ax.get_legend_handles_labels()
    assert any("default" in l for l in labels)
    assert any("winner" in l for l in labels)
    assert ax.get_xlim() == (0.0, 100.0)


def test_write_tune_pdf_includes_configurator_settings_section(tmp_path):
    settings_rows = [
        {"setting": "advance_level", "offset": 23, "default": 26, "best": 20},
        {"setting": "pwm_frequency", "offset": 24, "default": 24, "best": 48},
    ]
    p = report.write_tune_pdf(tmp_path, _manifest(), _result(),
                              [("advance_level", 26, 20)],
                              settings_rows=settings_rows)
    assert _is_pdf(p)


# --------------------------------------------------------------------------
# Raw data pages: wide (row-per-trial) layout instead of melted
# (row-per-metric) layout
# --------------------------------------------------------------------------
class _FakePdf:
    """Records each page's title + table headers instead of writing a real
    PDF, so tests can assert on layout without a PDF-parsing dependency."""

    def __init__(self):
        self.pages: list[tuple[str, list[str]]] = []

    def savefig(self, fig):
        title = fig.texts[0].get_text() if fig.texts else ""
        headers: list[str] = []
        if fig.axes and fig.axes[-1].tables:
            tbl = fig.axes[-1].tables[0]
            ncols = max(c for _, c in tbl.get_celld()) + 1
            headers = [tbl.get_celld()[(0, c)].get_text().get_text()
                      .replace("\n", "") for c in range(ncols)]
        self.pages.append((title, headers))


def _pyplot():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def _write_metrics(tmp_path, idx, summary=None, steady_points=None):
    d = tmp_path / "trials" / f"T{idx:03d}"
    d.mkdir(parents=True)
    (d / "metrics.json").write_text(json.dumps({
        "summary": summary or {}, "demag": {}, "startup": {},
        "steady_points": steady_points or [],
    }))
    return f"trials/T{idx:03d}"


def test_pdf_raw_data_pages_uses_wide_format_and_skips_empty_errors(tmp_path):
    plt = _pyplot()
    trials = []
    for i in range(3):
        d = _write_metrics(tmp_path, i, summary={f"m{j}": j for j in range(10)},
                           steady_points=[{"segment": "t50", "throttle": 0.5,
                                          "eff_gf_per_w": 3.0 + i}])
        trials.append({"index": i, "stage": "baseline", "kind": "baseline",
                       "overrides": {}, "dir": d})
    manifest = _manifest(trials=trials)

    fake_pdf = _FakePdf()
    report._pdf_raw_data_pages(plt, fake_pdf, tmp_path, manifest)
    plt.close("all")

    titles = [t for t, _ in fake_pdf.pages]
    assert not any("load errors" in t for t in titles)
    # 10 summary metrics / group size 6 -> 2 column-groups, 3 trials fit on
    # one page each -> exactly 2 "Run summary raw data" pages
    assert sum(t.startswith("Run summary raw data") for t in titles) == 2
    assert any(t.startswith("Steady-point raw data") for t in titles)
    # wide format: one row per trial, not one row per metric
    summary_headers = next(h for t, h in fake_pdf.pages
                           if t.startswith("Run summary raw data"))
    assert "metric" not in summary_headers and "value" not in summary_headers


def test_pdf_raw_data_pages_reports_load_errors_separately(tmp_path):
    plt = _pyplot()
    manifest = _manifest(trials=[
        {"index": 0, "stage": "baseline", "kind": "baseline",
         "overrides": {}, "dir": "trials/does-not-exist"},
    ])

    fake_pdf = _FakePdf()
    report._pdf_raw_data_pages(plt, fake_pdf, tmp_path, manifest)
    plt.close("all")

    titles = [t for t, _ in fake_pdf.pages]
    assert any("load errors" in t for t in titles)
    assert not any(t.startswith("Run summary raw data (") for t in titles)


def test_pdf_raw_data_pages_excludes_zc_phase_hist_from_steady_columns(tmp_path):
    plt = _pyplot()
    d = _write_metrics(tmp_path, 0, steady_points=[
        {"segment": "t50", "throttle": 0.5, "eff_gf_per_w": 3.0,
         "zc_phase_hist": list(range(30))},
    ])
    manifest = _manifest(trials=[{"index": 0, "stage": "baseline",
                                  "kind": "baseline", "overrides": {},
                                  "dir": d}])

    fake_pdf = _FakePdf()
    report._pdf_raw_data_pages(plt, fake_pdf, tmp_path, manifest)
    plt.close("all")

    steady_headers = {h for t, headers in fake_pdf.pages
                      if t.startswith("Steady-point raw data")
                      for h in headers}
    assert "zc_phase_hist" not in steady_headers
    assert "eff_gf_per_w" in steady_headers
