"""Absolute smoke_gates: catch stuck-rotor / never-spun free-run failures."""
from __future__ import annotations

from pathlib import Path

import pytest

from hwci import metrics as metricsmod
from hwci.config import Profile, Segment, SmokeGates, load_profile
from hwci.model import RunResult

RUNS = Path(__file__).resolve().parents[1] / "runs"


def _noprop_profile(**gate_overrides) -> Profile:
    gates = SmokeGates(
        enabled=True,
        min_throttle=0.55,
        min_running_fraction=0.90,
        max_bemf_latch_samples=0,
        require_esc_drive_states=True,
        min_esc_drive_fraction=0.90,
        max_illegal_edges=0,
        min_rpm={0.6: 15000.0, 0.8: 25000.0, 1.0: 30000.0},
    )
    for k, v in gate_overrides.items():
        setattr(gates, k, v)
    return Profile(
        name="synthetic_noprop",
        sample_rate_hz=100.0,
        segments=[
            Segment(label="hold60", throttle=0.6, duration_s=1.0, steady=True),
            Segment(label="hold80", throttle=0.8, duration_s=1.0, steady=True),
            Segment(label="hold100", throttle=1.0, duration_s=1.0, steady=True),
        ],
        smoke_gates=gates,
    )


def _rows_for_segments(seg_specs: list[tuple[str, float, int]], **cols):
    """Build rows: list of (segment, throttle, n_samples)."""
    rows = []
    t = 0.0
    for lab, thr, n in seg_specs:
        for i in range(n):
            row = {"t": t, "segment": lab, "throttle_cmd": thr}
            for k, v in cols.items():
                if callable(v):
                    row[k] = v(lab, thr, i)
                elif isinstance(v, dict):
                    row[k] = v.get(lab, v.get("_", 0))
                else:
                    row[k] = v
            rows.append(row)
            t += 0.01
    return rows


def test_profile_yaml_loads_smoke_gates():
    p = load_profile("noprop_smoke_100pct_3a")
    assert p.smoke_gates is not None
    assert p.smoke_gates.enabled
    assert p.smoke_gates.min_rpm[1.0] == 30000.0


def test_disabled_gates_return_none():
    p = _noprop_profile()
    p.smoke_gates.enabled = False
    rows = _rows_for_segments([("hold100", 1.0, 50)], perf_running=0)
    r = metricsmod.evaluate_smoke_gates(
        RunResult(rows=rows), p, demag={}, steady_points=[])
    assert r is None


def test_good_free_run_passes():
    p = _noprop_profile()
    rpm = {"hold60": 20000, "hold80": 30000, "hold100": 43000}
    rows = _rows_for_segments(
        [("hold60", 0.6, 40), ("hold80", 0.8, 40), ("hold100", 1.0, 50)],
        perf_running=1,
        perf_bemf_timeout=0,
        perf_esc_state=5,
        perf_esc_illegal_edge_count=0,
        stand_rpm=lambda lab, thr, i: rpm[lab],
    )
    pts = [
        {"segment": "hold60", "throttle": 0.6, "rpm": 20000},
        {"segment": "hold80", "throttle": 0.8, "rpm": 30000},
        {"segment": "hold100", "throttle": 1.0, "rpm": 43000},
    ]
    g = metricsmod.evaluate_smoke_gates(
        RunResult(rows=rows), p, demag={}, steady_points=pts)
    assert g is not None and g["passed"], g
    assert all(c["pass"] for c in g["checks"])


def test_stuck_latch_fails():
    p = _noprop_profile()
    rows = _rows_for_segments(
        [("hold100", 1.0, 50)],
        perf_running=0,
        perf_bemf_timeout=102,
        perf_esc_state=7,
        perf_esc_illegal_edge_count=0,
        stand_rpm=0.0,
    )
    pts = [{"segment": "hold100", "throttle": 1.0, "rpm": 0.0}]
    g = metricsmod.evaluate_smoke_gates(
        RunResult(rows=rows), p, demag={}, steady_points=pts)
    assert g is not None and not g["passed"]
    names = {c["name"]: c for c in g["checks"]}
    assert not names["bemf_stuck_latch"]["pass"]
    assert not names["running_fraction"]["pass"]
    assert not names["min_rpm@1"]["pass"]


def test_illegal_edges_fail():
    p = _noprop_profile()
    rows = _rows_for_segments(
        [("hold100", 1.0, 50)],
        perf_running=1,
        perf_bemf_timeout=0,
        perf_esc_state=5,
        perf_esc_illegal_edge_count=3,
        stand_rpm=43000.0,
    )
    pts = [{"segment": "hold100", "throttle": 1.0, "rpm": 43000}]
    g = metricsmod.evaluate_smoke_gates(
        RunResult(rows=rows), p, demag={}, steady_points=pts)
    assert g is not None and not g["passed"]
    assert any(c["name"] == "esc_illegal_edges" and not c["pass"]
               for c in g["checks"])


def test_missing_esc_state_skips_state_gate_but_keeps_others():
    p = _noprop_profile()
    rows = _rows_for_segments(
        [("hold100", 1.0, 50)],
        perf_running=1,
        perf_bemf_timeout=0,
        # no perf_esc_state / illegal columns
        stand_rpm=43000.0,
    )
    pts = [{"segment": "hold100", "throttle": 1.0, "rpm": 43000}]
    g = metricsmod.evaluate_smoke_gates(
        RunResult(rows=rows), p, demag={}, steady_points=pts)
    assert g is not None and g["passed"]
    names = [c["name"] for c in g["checks"]]
    assert "esc_drive_states" not in names
    assert "running_fraction" in names


@pytest.mark.parametrize("run_dir,expect_pass", [
    ("split-pr33-esc-sm-v2-100pct-2", True),   # good free-run
    ("split-pr33-esc-sm-v2-100pct-1", False),  # stuck latch
])
def test_real_bench_runs(run_dir, expect_pass):
    path = RUNS / run_dir
    if not path.is_dir():
        pytest.skip(f"bench run {run_dir} not present")
    result = RunResult.load(path)
    # Prefer profile_def from meta so segments match the capture
    from hwci.config import profile_from_dict
    pd = result.meta.get("profile_def")
    if pd:
        # Older runs lack smoke_gates in profile_def — inject current gates
        if not pd.get("smoke_gates"):
            pd = dict(pd)
            pd["smoke_gates"] = {
                "enabled": True,
                "min_throttle": 0.55,
                "min_running_fraction": 0.90,
                "max_bemf_latch_samples": 0,
                "require_esc_drive_states": True,
                "min_esc_drive_fraction": 0.90,
                "max_illegal_edges": 0,
                "min_rpm": {0.6: 15000, 0.8: 25000, 1.0: 30000},
            }
        # v2-1 has no esc_state column — disable state gate for fair fail reasons
        cols = result.rows[0].keys() if result.rows else []
        if "perf_esc_state" not in cols:
            pd["smoke_gates"]["require_esc_drive_states"] = False
            pd["smoke_gates"]["max_illegal_edges"] = None
        profile = profile_from_dict(pd)
    else:
        profile = load_profile("noprop_smoke_100pct_3a")
    m = metricsmod.compute(result, profile)
    g = m.get("smoke_gates")
    assert g is not None
    assert g["passed"] is expect_pass, g
