"""Auto settings tuner: find the best AM32 EEPROM settings for a motor/prop.

Given a motor/prop on the rig, ``hwci tune`` searches the AM32 settings space
(advance_level, pwm_frequency, variable_pwm, auto_advance, max_ramp,
minimum_duty_cycle, ...) for the combination that maximizes efficiency (g/W)
subject to hard constraints.

Package layout:

* :mod:`hwci.tuner.spec` -- strict YAML schema
* :mod:`hwci.tuner.profiles` -- probe / startup / step / ramp / min-duty profiles
* :mod:`hwci.tuner.objective` -- scoring and constraint checks
* :mod:`hwci.tuner.search` -- climb, normalize, pick_winner
* :mod:`hwci.tuner.ramp` -- mech step-response max_ramp physics
* :mod:`hwci.tuner.minduty` -- low-throttle crawl → minimum_duty_cycle
* :mod:`hwci.tuner.backends` -- sim and hardware trial backends
* :mod:`hwci.tuner.session` -- Tuner session, resume, stages, finals
* :mod:`hwci.tuner.report` -- markdown report builders
"""
from __future__ import annotations

from .backends import HwTuneBackend, SimTuneBackend, TuneBackend
from .minduty import (compute_min_duty, compute_min_duty_for_idle,
                      dshot_to_host_throttle, duty_counts_at_dshot,
                      host_throttle_to_dshot, sustain_dshot_from_rows,
                      sustain_throttle_from_rows)
from .objective import check_constraints, objective_score, startup_stats
from .profiles import (RAMP_TRANSIENT_MAX_CURRENT_A, high_throttle_profile,
                       min_duty_measure_profile, min_duty_verify_profile,
                       probe_profile, ramp_measure_profile, startup_profile,
                       step_profile)
from .ramp import compute_max_ramp, mech_ramp_stats
from .search import (argmax_value, candidate_metric, climb, drift_factor,
                     efficiency_argmax, median_of, normalize, pick_winner,
                     winner_reason)
from .report import (campaign_table_md, load_pilot_card, pilot_card_md,
                     write_pilot_card)
from .session import MANIFEST_VERSION, TrialPlan, TunePaused, Tuner
from .spec import (ConstraintsSpec, FinalsSpec, ObjectiveSpec, PackSpec,
                   ParamSpec, ProbeSpec, StageSpec, StartupSpec, ThermalSpec,
                   TuneSpec, TuneSpecError, load_tune_spec, tune_spec_from_dict)

# Back-compat aliases for private names from the pre-split monolith.
_argmax_value = argmax_value
_median_of = median_of

__all__ = [
    "MANIFEST_VERSION",
    "RAMP_TRANSIENT_MAX_CURRENT_A",
    "ConstraintsSpec",
    "FinalsSpec",
    "HwTuneBackend",
    "ObjectiveSpec",
    "PackSpec",
    "ParamSpec",
    "ProbeSpec",
    "SimTuneBackend",
    "StageSpec",
    "StartupSpec",
    "ThermalSpec",
    "TrialPlan",
    "TuneBackend",
    "TunePaused",
    "TuneSpec",
    "TuneSpecError",
    "Tuner",
    "argmax_value",
    "campaign_table_md",
    "candidate_metric",
    "check_constraints",
    "climb",
    "compute_max_ramp",
    "compute_min_duty",
    "compute_min_duty_for_idle",
    "drift_factor",
    "dshot_to_host_throttle",
    "duty_counts_at_dshot",
    "efficiency_argmax",
    "high_throttle_profile",
    "host_throttle_to_dshot",
    "load_pilot_card",
    "load_tune_spec",
    "mech_ramp_stats",
    "min_duty_measure_profile",
    "min_duty_verify_profile",
    "normalize",
    "objective_score",
    "pick_winner",
    "pilot_card_md",
    "probe_profile",
    "ramp_measure_profile",
    "startup_profile",
    "startup_stats",
    "step_profile",
    "sustain_dshot_from_rows",
    "sustain_throttle_from_rows",
    "tune_spec_from_dict",
    "winner_reason",
    "write_pilot_card",
]
