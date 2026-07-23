"""Auto-tuner core: end-to-end sim tunes (injected optimum), objective
weighting, constraint disqualification, anchor normalization, tie-breaks."""
import pytest
import yaml

from hwci.model import RunResult
from hwci.sim import MotorParams
from hwci.tuner import (ObjectiveSpec, SimTuneBackend, Tuner, TuneSpecError,
                        objective_score, startup_stats, startup_profile,
                        tune_spec_from_dict)


def small_spec(**extra) -> dict:
    d = {
        "name": "test",
        "probe": {"dwell_s": 1.0},
        "objective": {"min_power_w": 5.0, "noise_floor_pct": 0.5},
        "anchors_every": 4,
        "parameters": {
            "advance_level": {"values": [14, 18, 22, 26, 30, 34, 38],
                              "refine_step": 2},
            "pwm_frequency": {"values": [8, 16, 24, 48]},
            "variable_pwm": {"values": [0, 1, 2]},
            "auto_advance": {"values": [0, 1]},
            "max_ramp": {"values": [160, 80, 40]},
        },
        "stages": [
            {"name": "advance", "sweep": "advance_level"},
            {"name": "pwm", "sweep": "pwm_frequency",
             "fixed": {"variable_pwm": 0}},
        ],
        "finals": {"profile": "tune_probe", "repeats": 1,
                   "startup_check": False},
    }
    d.update(extra)
    return d


def make_backend(**params) -> SimTuneBackend:
    defaults = dict(pole_pairs=7, demag_prone=True, startup_fail_ref=100.0)
    defaults.update(params)
    return SimTuneBackend(motor_params=MotorParams(**defaults), noise=0.0)


def run_tune(tmp_path, spec_d, backend, **kw):
    spec = tune_spec_from_dict(spec_d)
    t = Tuner(spec, backend, tmp_path / "tune",
              spec_text=yaml.safe_dump(spec_d), no_prompt=True,
              log=lambda s: None, **kw)
    return t, t.run()


# --------------------------------------------------------------------------
# end-to-end: injected optimum is found
# --------------------------------------------------------------------------
def test_e2e_finds_injected_optimum(tmp_path):
    backend = make_backend(advance_optimum=27.0, pwm_optimum_khz=24.0)
    _, result = run_tune(tmp_path, small_spec(), backend)
    winner = result["winner_overrides"]
    # advance grid is every 4, refine step 2: winner within one refine step
    assert abs(winner["advance_level"] - 27) <= 2
    assert winner["pwm_frequency"] == 24


def test_e2e_confirms_a_real_improvement_and_writes_outputs(tmp_path):
    # optimum far from the default (26) so finals see a real paired delta
    backend = make_backend(advance_optimum=33.0)
    tuner, result = run_tune(tmp_path, small_spec(), backend)
    assert result["confirmed"]
    assert result["median_paired_delta"] > 0
    out = tmp_path / "tune"
    assert (out / "report.md").exists()
    assert (out / "settings_diff.md").exists()
    assert (out / "base_settings.bin").exists()
    best = (out / "best_settings.bin").read_bytes()
    assert best[23] == result["winner_overrides"]["advance_level"]
    # identity bytes preserved from the base page (never mutated)
    base = (out / "base_settings.bin").read_bytes()
    assert best[1] == base[1] and best[3:5] == base[3:5]


def test_e2e_default_optimum_keeps_default_settings(tmp_path):
    # optimum AT the default: winner is within noise of default; finals must
    # not confirm an improvement, and best_settings.bin stays the base page
    backend = make_backend(advance_optimum=26.0)
    _, result = run_tune(tmp_path, small_spec(), backend)
    out = tmp_path / "tune"
    if not result["confirmed"]:
        assert (out / "best_settings.bin").read_bytes() == \
            (out / "base_settings.bin").read_bytes()


# --------------------------------------------------------------------------
# constraint disqualification
# --------------------------------------------------------------------------
def test_constraint_only_stage_disqualifies_desyncing_ramp(tmp_path):
    spec_d = small_spec(stages=[
        {"name": "ramp", "sweep": "max_ramp", "constraint_only": True,
         "profile": "tune_step"},
    ])
    _, result = run_tune(tmp_path, spec_d, make_backend())
    # 160 desyncs on the step profile (disqualified: demag + bemf timeouts);
    # 80 is the first listed value that passes
    ledger = (tmp_path / "tune" / "manifest.json").read_text()
    assert "demag events" in ledger
    assert result["winner_overrides"]["max_ramp"] == 80


def test_disqualified_candidate_never_wins(tmp_path):
    tuner, _ = run_tune(tmp_path, small_spec(stages=[]), make_backend())
    cands = [
        {"overrides": {"advance_level": 30}, "order": 0, "value": 30,
         "entries": [{"score_raw": 99.0, "score_norm": 99.0,
                      "disqualified": ["demag events 1 > 0"],
                      "jitter_pct": 0.1, "fet_temp_c": None}]},
        {"overrides": {"advance_level": 22}, "order": 1, "value": 22,
         "entries": [{"score_raw": 5.0, "score_norm": 5.0,
                      "disqualified": None,
                      "jitter_pct": 0.5, "fet_temp_c": None}]},
    ]
    assert tuner._pick_winner(cands)["overrides"] == {"advance_level": 22}


def test_multi_repeat_single_fluke_dq_does_not_kill_candidate(tmp_path):
    """With repeats > 1, one disqualified leg must not erase a candidate that
    still has clean scores; the median of the clean legs is used."""
    tuner, _ = run_tune(tmp_path, small_spec(stages=[]), make_backend())
    tuner.spec.objective.noise_floor_pct = 0.5
    cands = [
        {"overrides": {"advance_level": 30}, "order": 0, "value": 30,
         "entries": [
             {"score_raw": 6.2, "score_norm": 6.2, "disqualified": None,
              "jitter_pct": 0.5, "fet_temp_c": 40.0},
             {"score_raw": None, "score_norm": None,
              "disqualified": ["demag events 1 > 0"],
              "jitter_pct": 9.0, "fet_temp_c": 80.0},
         ]},
        {"overrides": {"advance_level": 22}, "order": 1, "value": 22,
         "entries": [
             {"score_raw": 5.0, "score_norm": 5.0, "disqualified": None,
              "jitter_pct": 0.1, "fet_temp_c": 30.0},
             {"score_raw": 5.1, "score_norm": 5.1, "disqualified": None,
              "jitter_pct": 0.1, "fet_temp_c": 30.0},
         ]},
    ]
    assert tuner._pick_winner(cands)["overrides"] == {"advance_level": 30}
    assert tuner._pick_winner(cands)["score"] == pytest.approx(6.2)


def test_all_disqualified_yields_no_winner(tmp_path):
    tuner, _ = run_tune(tmp_path, small_spec(stages=[]), make_backend())
    cands = [{"overrides": {}, "order": 0, "value": 0,
              "entries": [{"score_raw": None, "score_norm": None,
                           "disqualified": ["run aborted: x"],
                           "jitter_pct": None, "fet_temp_c": None}]}]
    assert tuner._pick_winner(cands) is None


# --------------------------------------------------------------------------
# objective weighting
# --------------------------------------------------------------------------
def _metrics(points):
    return {"steady_points": [
        {"segment": lbl, "eff_gf_per_w": eff, "elec_power_w": pw}
        for lbl, eff, pw in points]}


def test_objective_default_noise_floor_matches_empirical_validation():
    # Locks in the 2026-07-07 value (tightened from 3.0): real anchor-to-anchor
    # score CV in completed sessions was 0.50%/0.77%, so 2.0 keeps a deliberate
    # ~2.6x margin over the worse of the two - see ObjectiveSpec.noise_floor_pct.
    assert ObjectiveSpec().noise_floor_pct == 2.0
    assert ObjectiveSpec().min_power_w == 20.0


def test_objective_weighted_mean():
    spec = tune_spec_from_dict({
        "name": "t",
        "objective": {"weights": {"a": 1.0, "b": 3.0}, "min_power_w": 10.0}})
    m = _metrics([("a", 4.0, 50.0), ("b", 8.0, 50.0)])
    assert objective_score(m, spec.objective) == pytest.approx(
        (1 * 4.0 + 3 * 8.0) / 4)


def test_objective_excludes_low_power_points():
    spec = tune_spec_from_dict({"name": "t"})
    m = _metrics([("t30", 40.0, 2.3),      # noise point, must not score
                  ("t50", 6.0, 50.0), ("t70", 5.0, 100.0)])
    assert objective_score(m, spec.objective) == pytest.approx(
        (2 * 6.0 + 1 * 5.0) / 3)


def test_objective_unlisted_label_gets_weight_one():
    spec = tune_spec_from_dict(
        {"name": "t", "objective": {"weights": {}, "min_power_w": 1.0}})
    m = _metrics([("x", 4.0, 50.0), ("y", 8.0, 50.0)])
    assert objective_score(m, spec.objective) == pytest.approx(6.0)


def test_objective_none_when_no_point_qualifies():
    spec = tune_spec_from_dict({"name": "t"})
    assert objective_score(_metrics([("t30", 40.0, 2.0)]),
                           spec.objective) is None


# --------------------------------------------------------------------------
# anchor normalization
# --------------------------------------------------------------------------
def test_drift_factor_interpolates_between_anchors():
    anchors = [(0, 10.0), (5, 8.0)]
    assert Tuner._drift_factor(anchors, 0) == pytest.approx(1.0)
    assert Tuner._drift_factor(anchors, 5) == pytest.approx(10.0 / 8.0)
    assert Tuner._drift_factor(anchors, 2) == pytest.approx(10.0 / 9.2)
    assert Tuner._drift_factor(anchors, 9) == pytest.approx(10.0 / 8.0)
    assert Tuner._drift_factor([], 3) == 1.0


def test_e2e_finds_optimum_under_injected_linear_drift(tmp_path):
    # The rig degrades linearly (sagging pack modeled as falling motor
    # efficiency): raw scores of later trials read lower. Anchor
    # normalization must cancel it so the argmax stays at the optimum.
    backend = make_backend(advance_optimum=27.0)

    def drift(index, plan):
        backend.sim.params.motor_efficiency = 0.82 * (1.0 - 0.004 * index)

    spec_d = small_spec(stages=[{"name": "advance",
                                 "sweep": "advance_level"}])
    _, result = run_tune(tmp_path, spec_d, backend, before_trial=drift)
    assert abs(result["winner_overrides"]["advance_level"] - 27) <= 2
    # anchors themselves normalize flat (same settings, drift cancelled)
    import json
    m = json.loads((tmp_path / "tune" / "manifest.json").read_text())
    anchors = [e for e in m["trials"]
               if e["stage"] == "advance" and e["kind"] == "anchor"]
    norms = [e["score_norm"] for e in anchors if e["score_norm"]]
    assert max(norms) - min(norms) <= 0.02 * norms[0]
    # ...while their raw scores visibly drifted
    raws = [e["score_raw"] for e in anchors]
    assert max(raws) - min(raws) > 0.02 * raws[0]


# --------------------------------------------------------------------------
# tie-break order: jitter, then FET temp, then closest-to-default
# --------------------------------------------------------------------------
def _cand(order, ov, score, jitter, fet):
    return {"overrides": ov, "order": order, "value": order,
            "entries": [{"score_raw": score, "score_norm": score,
                         "disqualified": None, "jitter_pct": jitter,
                         "fet_temp_c": fet}]}


def test_tie_breaks_on_jitter_first(tmp_path):
    tuner, _ = run_tune(tmp_path, small_spec(stages=[]), make_backend())
    tuner.spec.objective.noise_floor_pct = 3.0
    cands = [_cand(0, {"advance_level": 30}, 6.00, jitter=0.9, fet=40.0),
             _cand(1, {"advance_level": 34}, 6.05, jitter=0.5, fet=60.0)]
    assert tuner._pick_winner(cands)["overrides"] == {"advance_level": 34}


def test_tie_breaks_on_fet_temp_second(tmp_path):
    tuner, _ = run_tune(tmp_path, small_spec(stages=[]), make_backend())
    tuner.spec.objective.noise_floor_pct = 3.0
    cands = [_cand(0, {"advance_level": 30}, 6.00, jitter=0.5, fet=40.0),
             _cand(1, {"advance_level": 34}, 6.05, jitter=0.5, fet=60.0)]
    assert tuner._pick_winner(cands)["overrides"] == {"advance_level": 30}


def test_tie_breaks_on_distance_to_default_third(tmp_path):
    tuner, _ = run_tune(tmp_path, small_spec(stages=[]), make_backend())
    tuner.spec.objective.noise_floor_pct = 3.0
    # default advance_level is 26 (base page): 28 is closer than 34
    cands = [_cand(0, {"advance_level": 34}, 6.05, jitter=0.5, fet=40.0),
             _cand(1, {"advance_level": 28}, 6.00, jitter=0.5, fet=40.0)]
    assert tuner._pick_winner(cands)["overrides"] == {"advance_level": 28}


def test_outside_noise_floor_score_wins_regardless(tmp_path):
    tuner, _ = run_tune(tmp_path, small_spec(stages=[]), make_backend())
    tuner.spec.objective.noise_floor_pct = 3.0
    cands = [_cand(0, {"advance_level": 30}, 6.50, jitter=9.9, fet=99.0),
             _cand(1, {"advance_level": 34}, 6.00, jitter=0.1, fet=20.0)]
    assert tuner._pick_winner(cands)["overrides"] == {"advance_level": 30}


def test_stage_records_efficiency_argmax_when_tiebreak_differs(tmp_path):
    """When noise-floor tie-break picks a non-max-g/W candidate, the stage
    record must expose both the chosen winner and the pure efficiency argmax."""
    from hwci.tuner import efficiency_argmax, pick_winner, winner_reason
    cands = [
        _cand(0, {"advance_level": 30}, 6.00, jitter=0.9, fet=40.0),
        _cand(1, {"advance_level": 34}, 6.05, jitter=0.5, fet=60.0),
    ]
    w = pick_winner(cands, noise_floor_pct=3.0,
                    distance_fn=lambda ov: 0.0)
    a = efficiency_argmax(cands)
    assert a["overrides"] == {"advance_level": 34}
    assert w["overrides"] == {"advance_level": 34}  # jitter wins for 34
    # Force a tie-break that diverges: same score band, worse jitter on max
    cands2 = [
        _cand(0, {"advance_level": 30}, 6.00, jitter=0.2, fet=40.0),
        _cand(1, {"advance_level": 34}, 6.05, jitter=0.9, fet=40.0),
    ]
    w2 = pick_winner(cands2, noise_floor_pct=3.0,
                     distance_fn=lambda ov: abs(ov["advance_level"] - 26))
    a2 = efficiency_argmax(cands2)
    assert a2["overrides"] == {"advance_level": 34}
    assert w2["overrides"] == {"advance_level": 30}
    assert winner_reason(w2, a2) == "noise_floor_tiebreak:jitter"


def test_finals_min_delta_rejects_tiny_gain(tmp_path):
    """A tiny positive paired delta below min_delta_pct must not confirm."""
    # Optimum very close to default so ABBA Δ is tiny; min_delta_pct=5
    # should refuse confirmation even if Δ > 0.
    backend = make_backend(advance_optimum=27.0)  # default page is 26
    spec_d = small_spec(
        stages=[{"name": "advance", "sweep": "advance_level",
                 "search": "climb"}],
        finals={"profile": "tune_probe", "repeats": 1,
                "startup_check": False, "min_delta_pct": 50.0,
                "extra_repeats_if_close": 0})
    _, result = run_tune(tmp_path, spec_d, backend)
    # Either not confirmed, or if confirmed the delta cleared 50% (unlikely)
    if result["median_paired_delta"] is not None:
        thr = result["min_delta_threshold"]
        if result["median_paired_delta"] <= thr:
            assert not result["confirmed"]


def test_pilot_card_written(tmp_path):
    backend = make_backend(advance_optimum=33.0)
    _, result = run_tune(tmp_path, small_spec(), backend)
    out = tmp_path / "tune"
    assert (out / "pilot_card.md").exists()
    assert (out / "pilot_card.json").exists()
    import json
    card = json.loads((out / "pilot_card.json").read_text())
    assert card["confirmed"] == result["confirmed"]
    assert "winner_overrides" in card


def test_campaign_table_aggregates_sessions(tmp_path):
    from hwci.tuner import campaign_table_md, load_pilot_card
    backend = make_backend(advance_optimum=33.0)
    run_tune(tmp_path, small_spec(), backend)
    # second session
    run_tune(tmp_path / "b", small_spec(), make_backend(advance_optimum=26.0))
    # first run lands in tmp_path/tune; second needs out dir
    # re-run second properly
    from hwci.tuner import Tuner, tune_spec_from_dict
    import yaml
    d = small_spec()
    t = Tuner(tune_spec_from_dict(d), make_backend(advance_optimum=26.0),
              tmp_path / "tune2", spec_text=yaml.safe_dump(d),
              no_prompt=True, log=lambda s: None)
    t.run()
    c1 = load_pilot_card(tmp_path / "tune")
    c2 = load_pilot_card(tmp_path / "tune2")
    md = campaign_table_md([c1, c2])
    assert "Tune campaign summary" in md
    assert "Sessions: **2**" in md


def test_polish_radius_limits_values_tested(tmp_path):
    import json
    # polish stage only: incumbent default advance 26, radius 4 -> [22,26,30]
    # from the full grid (refine_step not applied on polish)
    spec_d = small_spec(stages=[
        {"name": "polish", "sweep": "advance_level", "search": "grid",
         "polish_radius": 4},
    ], finals={"profile": "tune_probe", "repeats": 1,
               "startup_check": False, "extra_repeats_if_close": 0,
               "min_delta_pct": 0.0})
    run_tune(tmp_path, spec_d, make_backend(advance_optimum=26.0))
    m = json.loads((tmp_path / "tune" / "manifest.json").read_text())
    vals = {e["overrides"].get("advance_level")
            for e in m["trials"]
            if e["stage"] == "polish" and e["kind"] == "trial"}
    assert vals  # at least incumbent
    assert all(abs(v - 26) <= 4 for v in vals if v is not None)
    assert 14 not in vals and 38 not in vals


# --------------------------------------------------------------------------
# startup stats (inline PR #22 stand-in)
# --------------------------------------------------------------------------
def test_startup_stats_counts_failed_cycles():
    spec = tune_spec_from_dict({"name": "t", "constraints": {
        "startup": {"cycles": 3, "spin_throttle": 0.15, "min_rpm": 1000}}})
    profile = startup_profile(spec)
    rows = []
    t = 0.0
    for seg in profile.segments:
        n = int(seg.duration_s * 100)
        for _ in range(n):
            rpm = 0.0
            if seg.label.startswith("spin"):
                # spin0 healthy, spin1 fails (never spins), spin2 healthy
                rpm = 0.0 if seg.label == "spin1" else 4000.0
            rows.append({"t": t, "segment": seg.label,
                         "stand_rpm": rpm, "perf_e_rpm": rpm * 7})
            t += 0.01
    result = RunResult(meta={"pole_pairs": 7}, rows=rows)
    st = startup_stats(result, profile, min_rpm=1000.0)
    assert st == {"cycles": 3, "failed": 1, "failed_segments": ["spin1"],
                  "min_rpm": 1000.0}


def test_startup_stats_uses_perf_erpm_when_stand_is_dead():
    spec = tune_spec_from_dict({"name": "t", "constraints": {
        "startup": {"cycles": 1}}})
    profile = startup_profile(spec)
    rows = [{"t": i * 0.01, "segment": "spin0", "stand_rpm": None,
             "perf_e_rpm": 4000 * 7} for i in range(150)]
    result = RunResult(meta={"pole_pairs": 7}, rows=rows)
    assert startup_stats(result, profile, min_rpm=1000.0)["failed"] == 0


# --------------------------------------------------------------------------
# probe/step profile safety shape (bench-learned: snap transients trip the
# 40 A harness limit - 43-75 A observed on a 0.5->0.7 snap on the 6S bench)
# --------------------------------------------------------------------------
def test_probe_profile_ramps_into_large_steps():
    from hwci.tuner import probe_profile
    spec = tune_spec_from_dict(small_spec(
        probe={"dwell_s": 1.0,
               "points": {"t30": 0.30, "t50": 0.50, "t70": 0.70}}))
    labels = [s.label for s in probe_profile(spec).segments]
    # w20->t30 is a 10% step: no ramp; the 20% steps each get one
    assert "r_t30" not in labels
    assert labels.index("r_t50") == labels.index("t50") - 1
    assert labels.index("r_t70") == labels.index("t70") - 1


def test_probe_profile_ramp_segments_are_not_steady():
    from hwci.tuner import probe_profile
    spec = tune_spec_from_dict(small_spec(
        probe={"dwell_s": 1.0, "points": {"t30": 0.30, "t70": 0.70}}))
    p = probe_profile(spec)
    r70 = next(s for s in p.segments if s.label == "r_t70")
    assert r70.ramp and not r70.steady and r70.throttle == 0.70


def test_default_tune_probe_yaml_ramps_into_large_steps():
    from hwci.tuner import probe_profile
    spec = tune_spec_from_dict(small_spec())   # no points override
    labels = [s.label for s in probe_profile(spec).segments]
    for pt in ("t50", "t70", "t90"):
        assert f"r_{pt}" in labels
    assert "r_t30" not in labels


def test_step_profile_current_limit_has_snap_headroom():
    from hwci.tuner import step_profile
    # Default probe envelope (no low max_rpm) keeps the classic 0.30→0.95 snap
    # and the full transient current budget.
    spec = tune_spec_from_dict(small_spec(
        probe={"dwell_s": 1.0,
               "safety": {"max_current_a": 40.0, "max_thrust_n": 16.0}}))
    p = step_profile(spec)
    from hwci.tuner import RAMP_TRANSIENT_MAX_CURRENT_A
    assert p.safety.max_current_a == RAMP_TRANSIENT_MAX_CURRENT_A
    assert p.safety.max_thrust_n == 16.0
    thr = [s.throttle for s in p.segments if s.label.startswith("snap")]
    assert thr and max(thr) >= 0.90
    spec_hi = tune_spec_from_dict(small_spec(
        probe={"dwell_s": 1.0,
               "safety": {"max_current_a": RAMP_TRANSIENT_MAX_CURRENT_A + 20}}))
    assert (step_profile(spec_hi).safety.max_current_a
            == RAMP_TRANSIENT_MAX_CURRENT_A + 20)


def test_step_profile_softens_snap_for_heavy_prop_envelope():
    """10\"-style probe safety must not still demand 0.30→0.95 snaps."""
    from hwci.tuner import step_profile
    spec = tune_spec_from_dict(small_spec(
        probe={"dwell_s": 1.0,
               "points": {"t20": 0.20, "t35": 0.35},
               "safety": {"max_current_a": 55.0, "max_thrust_n": 45.0,
                          "max_rpm": 9000}}))
    p = step_profile(spec)
    snaps = [s.throttle for s in p.segments if s.label.startswith("snap")]
    holds = [s.throttle for s in p.segments if s.label.startswith("hold")]
    assert snaps and max(snaps) <= 0.70
    assert holds and max(holds) <= 0.30
    assert max(snaps) - max(holds) >= 0.15


# --------------------------------------------------------------------------
# session end leaves the device on the verdict settings
# --------------------------------------------------------------------------
def test_device_left_on_base_page_when_not_confirmed(tmp_path):
    backend = make_backend(advance_optimum=26.0)   # optimum AT the default
    _, result = run_tune(tmp_path, small_spec(), backend)
    if not result["confirmed"]:
        base = (tmp_path / "tune" / "base_settings.bin").read_bytes()
        assert backend.read_page() == base


def test_device_left_on_winner_when_confirmed(tmp_path):
    backend = make_backend(advance_optimum=33.0)
    _, result = run_tune(tmp_path, small_spec(), backend)
    assert result["confirmed"]
    assert backend.read_page() == \
        (tmp_path / "tune" / "best_settings.bin").read_bytes()


# --------------------------------------------------------------------------
# hill-climb sweep (search: climb)
# --------------------------------------------------------------------------
def climb_spec(**extra):
    d = small_spec(stages=[
        {"name": "advance", "sweep": "advance_level", "search": "climb",
         "fixed": {}},
    ])
    d.update(extra)
    return d


def test_climb_finds_injected_optimum(tmp_path):
    backend = make_backend(advance_optimum=33.0)
    _, result = run_tune(tmp_path, climb_spec(), backend)
    # grid every 4 + refine step 2: within one refine step of the optimum
    assert abs(result["winner_overrides"]["advance_level"] - 33) <= 2


def test_climb_tests_fewer_values_than_grid(tmp_path):
    import json
    backend = make_backend(advance_optimum=33.0)
    run_tune(tmp_path, climb_spec(), backend)
    m = json.loads((tmp_path / "tune" / "manifest.json").read_text())
    swept = {e["overrides"].get("advance_level")
             for e in m["trials"]
             if e["stage"] == "advance" and e["kind"] == "trial"}
    # 7-value grid (+2 refine): the climb from the default (26) toward 33
    # must never visit the far low end
    assert 14 not in swept and 18 not in swept
    assert len(swept) < 7


def test_climb_walks_downhill_direction_too(tmp_path):
    # optimum BELOW the default: first (upward) direction fails immediately,
    # the climb must then walk down and still find it
    backend = make_backend(advance_optimum=17.0)
    _, result = run_tune(tmp_path, climb_spec(), backend)
    assert abs(result["winner_overrides"]["advance_level"] - 17) <= 3


def test_climb_rejected_for_ab_and_constraint_stages():
    with pytest.raises(TuneSpecError, match="climb"):
        tune_spec_from_dict(small_spec(stages=[
            {"name": "m", "ab_candidates": [{}, {"variable_pwm": 1}],
             "search": "climb"}]))
    with pytest.raises(TuneSpecError, match="climb"):
        tune_spec_from_dict(small_spec(stages=[
            {"name": "r", "sweep": "max_ramp", "constraint_only": True,
             "search": "climb"}]))
    with pytest.raises(TuneSpecError, match="search"):
        tune_spec_from_dict(small_spec(stages=[
            {"name": "a", "sweep": "advance_level", "search": "bogus"}]))


# --------------------------------------------------------------------------
# baseline health gate: a session must not run against a broken reference
# --------------------------------------------------------------------------
def bad_safety_spec():
    # impossible current limit: every run aborts -> baseline disqualified
    return small_spec(probe={"dwell_s": 1.0,
                             "safety": {"max_current_a": 0.001}})


def test_baseline_disqualified_twice_pauses_session(tmp_path):
    import json
    from hwci.tuner import TunePaused
    with pytest.raises(TunePaused, match="baseline"):
        run_tune(tmp_path, bad_safety_spec(), make_backend())
    m = json.loads((tmp_path / "tune" / "manifest.json").read_text())
    # ledger quarantined so a resume re-runs the baseline fresh, and no
    # jitter reference was taken from the disqualified runs
    assert m["trials"] == []
    assert m["jitter_reference"] is None


def test_baseline_pause_then_resume_completes(tmp_path):
    from hwci.tuner import TunePaused
    with pytest.raises(TunePaused):
        run_tune(tmp_path, bad_safety_spec(), make_backend())
    # "fix the limits", then resume the same session directory
    spec = tune_spec_from_dict(small_spec())
    t = Tuner(spec, make_backend(), tmp_path / "tune",
              no_prompt=True, resume=True, log=lambda s: None)
    result = t.run()
    assert result["winner_overrides"] is not None
    import json
    m = json.loads((tmp_path / "tune" / "manifest.json").read_text())
    assert m["jitter_reference"] is not None


# --------------------------------------------------------------------------
# mech-ramp measure stage (measure: ramp_rate)
# --------------------------------------------------------------------------
def _step_rows(tau_s=0.08, base_rpm=60000.0, hi_rpm=140000.0,
               base_i=3.0, pk_i=20.0, hi_i=8.0, dt=0.005):
    """Synthetic tune_ramp_measure rows: first-order eRPM rise, current
    spike decaying with the same tau."""
    import math
    rows = []
    t = 0.0
    def emit(seg, thr, dur, rpm_fn, i_fn):
        nonlocal t
        t0 = t
        while t < t0 + dur:
            rows.append({"t": t, "segment": seg, "throttle_cmd": thr,
                         "perf_e_rpm": rpm_fn(t - t0),
                         "stand_current_a": i_fn(t - t0)})
            t += dt
    emit("hold_lo", 0.20, 1.0, lambda dt_: base_rpm, lambda dt_: base_i)
    emit("step_up", 0.55, 2.0,
         lambda dt_: base_rpm + (hi_rpm - base_rpm) * (1 - math.exp(-dt_ / tau_s)),
         lambda dt_: hi_i + (pk_i - hi_i) * math.exp(-dt_ / tau_s))
    emit("drop", 0.20, 1.0, lambda dt_: base_rpm, lambda dt_: base_i)
    emit("step_up2", 0.55, 2.0,
         lambda dt_: base_rpm + (hi_rpm - base_rpm) * (1 - math.exp(-dt_ / tau_s)),
         lambda dt_: hi_i + (pk_i - hi_i) * math.exp(-dt_ / tau_s))
    return rows


def test_mech_ramp_stats_recovers_plant_constants():
    from hwci.tuner import mech_ramp_stats
    s = mech_ramp_stats(_step_rows(tau_s=0.08))
    assert s is not None
    assert 60 <= s["tau_ms"] <= 100          # 80 ms +/- sampling grain
    # k = (peak - base) / step_pct = (20 - 3) / 35
    assert s["k_a_per_pct"] == pytest.approx(17.0 / 35.0, rel=0.15)
    assert s["rpm_hi"] > s["rpm_lo"]


def test_mech_ramp_stats_none_without_a_real_step():
    from hwci.tuner import mech_ramp_stats
    flat = [{"t": i * 0.005, "segment": s, "throttle_cmd": 0.2,
             "perf_e_rpm": 60000.0, "stand_current_a": 3.0}
            for s in ("hold_lo", "step_up") for i in range(100)]
    assert mech_ramp_stats(flat) is None


def test_compute_max_ramp_math_and_clamping():
    from hwci.tuner import compute_max_ramp
    stats = {"tau_ms": 50.0, "k_a_per_pct": 0.5}
    # lead = 30/0.5 = 60%; rate = 60/50 = 1.2 %/ms -> 12 in 0.1%/ms units
    assert compute_max_ramp(stats, current_budget_a=30.0, lo=1, hi=160,
                            margin=1.0) == 12
    assert compute_max_ramp(stats, current_budget_a=30.0, lo=1, hi=160,
                            margin=0.5) == 6
    assert compute_max_ramp(stats, current_budget_a=1e9, lo=1, hi=160,
                            margin=1.0) == 160    # clamped to field max
    assert compute_max_ramp(stats, current_budget_a=0.0, lo=4, hi=160,
                            margin=1.0) == 4      # clamped to field min


def test_measure_stage_spec_validation():
    with pytest.raises(TuneSpecError, match="exactly one"):
        tune_spec_from_dict(small_spec(stages=[
            {"name": "r", "measure": "ramp_rate", "sweep": "max_ramp"}]))
    with pytest.raises(TuneSpecError, match="ramp_rate|min_duty"):
        tune_spec_from_dict(small_spec(stages=[
            {"name": "r", "measure": "bogus"}]))
    with pytest.raises(TuneSpecError, match="margin"):
        tune_spec_from_dict(small_spec(stages=[
            {"name": "r", "measure": "ramp_rate", "margin": 5.0}]))
    # min_duty is a first-class measure mode
    spec = tune_spec_from_dict(small_spec(stages=[
        {"name": "md", "measure": "min_duty", "margin": 1.15}]))
    assert spec.stages[0].measure == "min_duty"


def test_compute_min_duty_math_and_clamping():
    from hwci.tuner import compute_min_duty, compute_min_duty_for_idle
    # Deprecated host-throttle form (kept for back-compat).
    assert compute_min_duty(0.029, lo=1, hi=50, margin=1.0) == 6
    assert compute_min_duty(0.029, lo=1, hi=50, margin=1.15) == 7
    assert compute_min_duty(0.0, lo=1, hi=50, margin=1.15) == 1
    assert compute_min_duty(0.5, lo=1, hi=50, margin=1.0) == 50
    # Idle-targeted: plant needs duty at DShot 65 with floor=1 (~28 counts
    # is low); real cliff at DShot 88 (~duty 51) with margin 1.15 → ~6.
    assert compute_min_duty_for_idle(88, lo=1, hi=50, margin=1.0,
                                    measure_eeprom=1) == 6
    assert compute_min_duty_for_idle(88, lo=1, hi=50, margin=1.15,
                                    measure_eeprom=1) == 6
    # User observation: cliff ~DShot 55 with higher floor maps to need ~58
    # counts; targeting DShot 48 with margin → 7.
    # duty(55, measure_s=5) is not what measure uses; with measure_s=1
    # duty(55)≈18 → small S. The measure finds the cliff under floor=1.
    s = compute_min_duty_for_idle(65, lo=1, hi=50, margin=1.15,
                                 measure_eeprom=1)
    assert 1 <= s <= 10


def test_sustain_dshot_from_rows_picks_lowest_passing_hold():
    from hwci.tuner import dshot_to_host_throttle, sustain_dshot_from_rows
    rows = []
    # (dshot, stand_rpm, e_rpm, current) — d48 coasts (no current / eRPM
    # collapse) must fail; d55 is the lowest real sustain.
    for dshot, rpm, e_rpm, cur, n in (
            (100, 2000, 14000, 0.3, 20),
            (65, 900, 6300, 0.15, 20),
            (55, 700, 4900, 0.1, 20),
            (48, 450, 500, 0.0, 20)):   # coast / false-lock
        thr = dshot_to_host_throttle(dshot)
        for i in range(n):
            rows.append({
                "segment": f"d{dshot}",
                "throttle_cmd": thr,
                "stand_rpm": rpm,
                "stand_current_a": cur,
                "perf_e_rpm": e_rpm,
            })
    stats = sustain_dshot_from_rows(rows, min_rpm=400.0, pole_pairs=7)
    assert stats is not None
    assert stats["sustain_dshot"] == pytest.approx(55, abs=1)
    assert stats["failed_holds"] == 1


def test_e2e_measure_stage_sets_max_ramp(tmp_path):
    import json
    spec_d = small_spec(stages=[
        {"name": "ramp", "measure": "ramp_rate", "margin": 0.8}])
    backend = make_backend(demag_prone=False)
    _, result = run_tune(tmp_path, spec_d, backend)
    m = json.loads((tmp_path / "tune" / "manifest.json").read_text())
    st = m["stages"]["ramp"]
    assert st["measured"] is not None
    assert st["measured"]["tau_ms"] > 0
    assert st["computed_max_ramp"] is not None
    from hwci.settings import resolve_field
    f = resolve_field("max_ramp", None)
    assert f.lo <= st["computed_max_ramp"] <= f.hi
    # verify trial ran and the incumbent picked up the winner
    kinds = [t["kind"] for t in m["trials"] if t["stage"] == "ramp"]
    assert kinds[0] == "measure" and "verify" in kinds
    if st["winner"] is not None:
        assert m["incumbent"]["max_ramp"] == st["winner"]["max_ramp"]


def test_e2e_measure_stage_sets_minimum_duty_cycle(tmp_path):
    """Plant needs ~3% effective duty; idle-targeted measure should raise
    the floor so DShot 48 sustains and verify passes."""
    import json
    spec_d = small_spec(stages=[
        {"name": "min_duty", "measure": "min_duty", "margin": 1.15}])
    # 2% plant need: with floor=1, DShot ≳ 48+0.02*1999 ≈ 88 sustains in sim
    # (host-throttle model); idle verify requires floor ≥ 4 (S/200).
    backend = make_backend(demag_prone=False, sustain_throttle=0.02)
    _, result = run_tune(tmp_path, spec_d, backend)
    m = json.loads((tmp_path / "tune" / "manifest.json").read_text())
    st = m["stages"]["min_duty"]
    assert st["measured"] is not None
    # Measure finds a cliff above DShot idle; computed floor targets DShot 48.
    assert st["measured"]["sustain_dshot"] >= 48
    assert st["computed_minimum_duty_cycle"] is not None
    from hwci.settings import resolve_field
    f = resolve_field("minimum_duty_cycle", None)
    assert f.lo <= st["computed_minimum_duty_cycle"] <= f.hi
    # Floor must clear plant need at DShot 48 (eeprom units ≥ ~4–6).
    assert st["computed_minimum_duty_cycle"] >= 4
    kinds = [t["kind"] for t in m["trials"] if t["stage"] == "min_duty"]
    assert kinds[0] == "measure" and "verify" in kinds
    assert st["winner"] is not None
    assert m["incumbent"]["minimum_duty_cycle"] == st["winner"]["minimum_duty_cycle"]
    assert st["winner"]["minimum_duty_cycle"] >= st["computed_minimum_duty_cycle"]


def test_measure_stage_falls_back_to_bubble_search_when_measurement_desyncs(
        tmp_path):
    """A disqualified measurement run must not give up: bubble-search
    max_ramp from the floor (not thrash down from the ceiling)."""
    import json
    spec_d = small_spec(stages=[
        {"name": "ramp", "measure": "ramp_rate", "margin": 0.8}])
    # demag_step_threshold scaled so unrestricted (max_ramp=255) snaps
    # desync; safe once max_ramp is low enough that
    # jump 0.65 < 0.2*(160/max_ramp)  => max_ramp <= 49 (sim.py).
    backend = make_backend(demag_prone=True, demag_step_threshold=0.2,
                           demag_current_a=1.0)
    _, result = run_tune(tmp_path, spec_d, backend)
    m = json.loads((tmp_path / "tune" / "manifest.json").read_text())
    st = m["stages"]["ramp"]
    assert st["measured"] is None          # measurement itself desynced
    assert st["computed_max_ramp"] is None  # no physics to compute from
    assert st.get("search") == "bubble"
    trials = [t for t in m["trials"] if t["stage"] == "ramp"]
    assert trials[0]["kind"] == "measure" and trials[0]["disqualified"]
    verifies = [t for t in trials if t["kind"] == "verify"]
    assert len(verifies) > 1
    # Floor-first: first verify is the field lo (1), never the ceiling.
    assert verifies[0]["overrides"]["max_ramp"] == 1
    assert st["winner"] is not None
    mr = st["winner"]["max_ramp"]
    assert 1 <= mr <= 49
    assert m["incumbent"]["max_ramp"] == mr
    # Never probe above the first failure with a high-first 0.7 ladder.
    assert max(t["overrides"]["max_ramp"] for t in verifies) <= 64


def _spec_with_advance_and_ramp(**ramp_kwargs) -> dict:
    """A spec with a real 'advance' StageSpec (sweep=advance_level) ahead of
    the ramp stage, so _ramp_fallback_candidates can find it by identity -
    matching what the shipped tune specs actually look like. Tests still
    drive the ramp stage directly (t._run_measure_stage(...)); the advance
    stage is never actually executed, only its presence in spec.stages
    matters, plus whatever fabricated ledger history a test supplies."""
    return small_spec(stages=[
        {"name": "advance", "sweep": "advance_level"},
        {"name": "ramp", "measure": "ramp_rate", **ramp_kwargs}])


def _ledger_entry(idx, stage, kind, overrides, score_norm=None,
                   disqualified=None, discarded=False):
    return {"index": idx, "stage": stage, "kind": kind,
            "overrides": overrides, "score_norm": score_norm,
            "disqualified": disqualified, "discarded": discarded}


def test_ramp_fallback_candidates_ranks_by_score_and_excludes_incumbent(
        tmp_path):
    """Pure logic test of the candidate generator: alternative advance_level
    values from an already-run 'advance' stage, ranked best-score first,
    excluding whatever is already the (failing) incumbent, with pure
    firmware defaults appended last."""
    import yaml
    spec_d = _spec_with_advance_and_ramp()
    spec = tune_spec_from_dict(spec_d)
    t = Tuner(spec, make_backend(), tmp_path / "tune",
              spec_text=yaml.safe_dump(spec_d), no_prompt=True,
              log=lambda s: None)
    t.manifest["incumbent"] = {"advance_level": 18}
    t.manifest["trials"] = [
        _ledger_entry(0, "advance", "trial", {"advance_level": 18}, 3.0),
        _ledger_entry(1, "advance", "trial", {"advance_level": 26}, 2.8),
        _ledger_entry(2, "advance", "trial", {"advance_level": 34}, 2.9),
        # disqualified trials must not become candidates
        _ledger_entry(3, "advance", "trial", {"advance_level": 14},
                      disqualified=["run aborted: safety"]),
    ]
    cands = t._ramp_fallback_candidates()
    # 34 (2.9) outranks 26 (2.8); 18 excluded (== incumbent); 14 excluded
    # (disqualified); untested values (22, 30, 38) excluded (no score);
    # pure defaults appended last.
    assert cands == [{"advance_level": 34}, {"advance_level": 26}, {}]


def test_ramp_fallback_candidates_uses_median_across_repeats(tmp_path):
    """Ranking must match _pick_winner's own convention for the coordinate
    search it's re-deriving alternatives from: the median across ALL of a
    value's "trial"-kind entries, not whichever entry happens to be first
    in the ledger. A single noisy sample must not outrank the true median."""
    import yaml
    spec_d = _spec_with_advance_and_ramp()
    spec = tune_spec_from_dict(spec_d)
    t = Tuner(spec, make_backend(), tmp_path / "tune",
              spec_text=yaml.safe_dump(spec_d), no_prompt=True,
              log=lambda s: None)
    t.manifest["incumbent"] = {"advance_level": 18}
    t.manifest["trials"] = [
        _ledger_entry(0, "advance", "trial", {"advance_level": 18}, 3.0),
        # 26's median is 2.8 (repeats 3.1, 2.8, 2.5) despite a bad first
        # sample below 22's single sample of 2.6.
        _ledger_entry(1, "advance", "trial", {"advance_level": 26}, 3.1),
        _ledger_entry(2, "advance", "trial", {"advance_level": 22}, 2.6),
        _ledger_entry(3, "advance", "trial", {"advance_level": 26}, 2.8),
        _ledger_entry(4, "advance", "trial", {"advance_level": 26}, 2.5),
    ]
    cands = t._ramp_fallback_candidates()
    assert cands == [{"advance_level": 26}, {"advance_level": 22}, {}]


def test_ramp_fallback_candidates_keeps_value_with_single_fluke_dq(
        tmp_path):
    """Softer multi-repeat DQ: a single fluke demag on one repeat must not
    erase a value that still has a clean, scored leg. Score is the median of
    clean entries only (here just 3.1). All-DQ values are still excluded."""
    import yaml
    spec_d = _spec_with_advance_and_ramp()
    spec = tune_spec_from_dict(spec_d)
    t = Tuner(spec, make_backend(), tmp_path / "tune",
              spec_text=yaml.safe_dump(spec_d), no_prompt=True,
              log=lambda s: None)
    t.manifest["incumbent"] = {"advance_level": 18}
    t.manifest["trials"] = [
        _ledger_entry(0, "advance", "trial", {"advance_level": 18}, 3.0),
        _ledger_entry(1, "advance", "trial", {"advance_level": 26}, 3.1),
        _ledger_entry(2, "advance", "trial", {"advance_level": 26},
                      disqualified=["demag events 1 > 0"]),
        # fully DQ'd value stays out
        _ledger_entry(3, "advance", "trial", {"advance_level": 14},
                      disqualified=["demag events 1 > 0"]),
    ]
    cands = t._ramp_fallback_candidates()
    assert cands == [{"advance_level": 26}, {}]


def test_ramp_fallback_candidates_robust_to_fixed_on_advance_stage(
        tmp_path):
    """The advance stage may pin an unrelated setting during its own sweep
    (the same idiom the shipped pwm stage already uses, fixed={variable_pwm:
    0}) - matching must key off (stage identity, advance_level value), not
    exact-equality on the whole recorded overrides dict, or a `fixed` entry
    on the advance stage would silently zero out every candidate."""
    import yaml
    spec_d = small_spec(stages=[
        {"name": "advance", "sweep": "advance_level",
         "fixed": {"variable_pwm": 0}},
        {"name": "ramp", "measure": "ramp_rate"}])
    spec = tune_spec_from_dict(spec_d)
    t = Tuner(spec, make_backend(), tmp_path / "tune",
              spec_text=yaml.safe_dump(spec_d), no_prompt=True,
              log=lambda s: None)
    t.manifest["incumbent"] = {"advance_level": 18, "variable_pwm": 0}
    t.manifest["trials"] = [
        _ledger_entry(0, "advance", "trial",
                      {"advance_level": 18, "variable_pwm": 0}, 3.0),
        _ledger_entry(1, "advance", "trial",
                      {"advance_level": 26, "variable_pwm": 0}, 2.8),
    ]
    cands = t._ramp_fallback_candidates()
    assert cands == [{"advance_level": 26, "variable_pwm": 0}, {}]


def test_measure_stage_falls_back_to_alternative_advance_level(tmp_path):
    """If NO max_ramp certifies for the coordinate search's efficiency
    winner, the ramp stage must not just report failure and keep it: it
    should retry against progressively lower-ranked (but already-scored)
    advance_level alternatives and adopt the first one that IS ramp-safe."""
    import yaml
    spec_d = _spec_with_advance_and_ramp()
    spec = tune_spec_from_dict(spec_d)
    t = Tuner(spec, make_backend(), tmp_path / "tune",
              spec_text=yaml.safe_dump(spec_d), no_prompt=True,
              log=lambda s: None)
    # Fabricate coordinate-search history, as if an "advance" stage already
    # ran and settled on 18 (best score) with 26 as runner-up.
    t.manifest["incumbent"] = {"advance_level": 18}
    t.manifest["trials"] = [
        _ledger_entry(0, "advance", "trial", {"advance_level": 18}, 3.0),
        _ledger_entry(1, "advance", "trial", {"advance_level": 26}, 2.8),
    ]

    def fake_trial(plan):
        idx = len(t.manifest["trials"])
        ov = plan.overrides
        dq = None
        if plan.kind == "measure":
            dq = ["demag events 1 > 0"]        # unrestricted snap desyncs
        elif plan.kind == "verify":
            # advance_level 18 (the efficiency winner) NEVER certifies at
            # any max_ramp; 26 certifies for max_ramp <= 80.
            if ov.get("advance_level") == 18:
                dq = ["demag events 1 > 0"]
            elif ov.get("advance_level") == 26 and ov["max_ramp"] > 80:
                dq = ["demag events 1 > 0"]
        entry = {"index": idx, "stage": plan.stage, "kind": plan.kind,
                 "overrides": ov, "disqualified": dq, "discarded": False,
                 "dir": f"trials/T{idx:03d}"}
        t.manifest["trials"].append(entry)
        return entry

    t._trial = fake_trial
    ramp_stage = next(s for s in spec.stages if s.name == "ramp")
    t._run_measure_stage(ramp_stage)

    st = t.manifest["stages"]["ramp"]
    assert st["used_fallback_settings"] == {"advance_level": 26}
    # Bubble from floor + binary refine finds the largest safe value (80),
    # not the old 0.7-backoff ladder's first hit (60).
    assert st["winner"] == {"advance_level": 26, "max_ramp": 80}
    assert t.manifest["incumbent"] == {"advance_level": 26, "max_ramp": 80}


def test_measure_stage_fallback_verifies_exactly_what_it_adopts(tmp_path):
    """The settings combo that passes step_profile's snap verification must
    be EXACTLY the combo adopted as the incumbent (and later shipped) - not
    a combo where the ramp stage's own `fixed` silently wins in one merge
    and loses in the other. Give the ramp stage a `fixed` that conflicts
    with what the fallback candidate would otherwise carry, and confirm the
    ramp stage's own fixed value is what's both verified and adopted."""
    import yaml
    spec_d = _spec_with_advance_and_ramp(fixed={"pwm_frequency": 8})
    spec = tune_spec_from_dict(spec_d)
    t = Tuner(spec, make_backend(), tmp_path / "tune",
              spec_text=yaml.safe_dump(spec_d), no_prompt=True,
              log=lambda s: None)
    t.manifest["incumbent"] = {"advance_level": 18, "pwm_frequency": 24}
    t.manifest["trials"] = [
        _ledger_entry(0, "advance", "trial", {"advance_level": 18}, 3.0),
        _ledger_entry(1, "advance", "trial", {"advance_level": 26}, 2.8),
    ]
    seen_pwm_frequency_at_certify: list[int] = []

    def fake_trial(plan):
        idx = len(t.manifest["trials"])
        ov = plan.overrides
        dq = ["demag events 1 > 0"] if ov.get("advance_level") == 18 else None
        if plan.kind == "verify" and dq is None:
            seen_pwm_frequency_at_certify.append(ov.get("pwm_frequency"))
        entry = {"index": idx, "stage": plan.stage, "kind": plan.kind,
                 "overrides": ov, "disqualified": dq, "discarded": False,
                 "dir": f"trials/T{idx:03d}"}
        t.manifest["trials"].append(entry)
        return entry

    t._trial = fake_trial
    ramp_stage = next(s for s in spec.stages if s.name == "ramp")
    t._run_measure_stage(ramp_stage)

    # stage.fixed (pwm_frequency=8) must have been the value actually
    # snap-tested on every verify ...
    assert seen_pwm_frequency_at_certify
    assert all(x == 8 for x in seen_pwm_frequency_at_certify)
    # ... and the SAME value the session adopts going forward.
    assert t.manifest["incumbent"]["pwm_frequency"] == 8


def test_climb_uses_normalized_scores_under_drift(tmp_path):
    """Climb direction must follow anchor-normalized scores, not raw: under
    a strong efficiency drift, raw scores fall even for better settings, so
    a raw-only climb would stop early or walk the wrong way."""
    backend = make_backend(advance_optimum=33.0)

    def drift(index, plan):
        # Stronger than the e2e drift test: enough that a later better
        # setting can still look worse in raw space than an early worse one.
        backend.sim.params.motor_efficiency = 0.82 * (1.0 - 0.012 * index)

    _, result = run_tune(
        tmp_path, climb_spec(anchors_every=2), backend, before_trial=drift)
    assert abs(result["winner_overrides"]["advance_level"] - 33) <= 2


def test_high_throttle_failure_unconfirms_winner(tmp_path):
    """A winner that desyncs on the optional high-throttle hold must not be
    confirmed, even with a positive ABBA paired delta."""
    import json
    from hwci.tuner import high_throttle_profile

    spec_d = small_spec(
        stages=[{"name": "advance", "sweep": "advance_level",
                 "search": "climb"}],
        finals={"profile": "tune_probe", "repeats": 1,
                "startup_check": False, "high_throttle": 0.70,
                "high_throttle_dwell_s": 1.0})
    backend = make_backend(advance_optimum=33.0)
    # Inject: every high-throttle trial looks like a demag abort by
    # lowering the safety current limit only for that profile. Easier:
    # monkeypatch _trial to DQ final_high_throttle after a normal run.
    spec = tune_spec_from_dict(spec_d)
    t = Tuner(spec, backend, tmp_path / "tune",
              spec_text=yaml.safe_dump(spec_d), no_prompt=True,
              log=lambda s: None)
    real_trial = t._trial

    def wrap(plan):
        e = real_trial(plan)
        if plan.kind == "final_high_throttle":
            e["disqualified"] = ["demag events 1 > 0"]
            e["score_raw"] = None
            e["score_norm"] = None
        return e

    t._trial = wrap
    result = t.run()
    assert result["high_throttle"] is not None
    assert result["high_throttle"]["ok"] is False
    assert result["confirmed"] is False
    # device left on defaults
    base = (tmp_path / "tune" / "base_settings.bin").read_bytes()
    assert backend.read_page() == base
    assert high_throttle_profile(spec, 0.70, 1.0).name == "tune_high_throttle"
    m = json.loads((tmp_path / "tune" / "manifest.json").read_text())
    kinds = [e["kind"] for e in m["trials"] if e["stage"] == "finals"]
    assert "final_high_throttle" in kinds


def test_measure_stage_resets_incumbent_to_default_when_nothing_certifies(
        tmp_path):
    """If the efficiency winner AND every fallback candidate (including
    pure firmware defaults) fail to certify a safe max_ramp, the session
    must not silently keep the (proven-unsafe) efficiency winner as the
    incumbent - finals would then confirm and ship it despite never
    re-testing the throttle range that broke it. Resetting to {} makes
    finals compare default-vs-default (unconfirmable), so the session
    ships untouched firmware defaults instead."""
    import yaml
    spec_d = _spec_with_advance_and_ramp()
    spec = tune_spec_from_dict(spec_d)
    t = Tuner(spec, make_backend(), tmp_path / "tune",
              spec_text=yaml.safe_dump(spec_d), no_prompt=True,
              log=lambda s: None)
    t.manifest["incumbent"] = {"advance_level": 18}
    t.manifest["trials"] = [
        _ledger_entry(0, "advance", "trial", {"advance_level": 18}, 3.0),
        _ledger_entry(1, "advance", "trial", {"advance_level": 26}, 2.8),
    ]

    def fake_trial_always_desyncs(plan):
        idx = len(t.manifest["trials"])
        entry = {"index": idx, "stage": plan.stage, "kind": plan.kind,
                 "overrides": plan.overrides,
                 "disqualified": ["demag events 1 > 0"], "discarded": False,
                 "dir": f"trials/T{idx:03d}"}
        t.manifest["trials"].append(entry)
        return entry

    t._trial = fake_trial_always_desyncs
    ramp_stage = next(s for s in spec.stages if s.name == "ramp")
    t._run_measure_stage(ramp_stage)

    st = t.manifest["stages"]["ramp"]
    assert st["winner"] is None
    assert st["used_fallback_settings"] is None
    assert t.manifest["incumbent"] == {}
