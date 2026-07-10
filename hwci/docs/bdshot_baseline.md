# BDShot docs moved

BDShot / ARK FPV work is **SETUP B** only:

* Overview of both benches: [BENCH_SETUPS.md](BENCH_SETUPS.md)
* PX4 BDShot procedure: [setup_px4_bdshot.md](setup_px4_bdshot.md)
* Rig file: `config/rig.px4_bdshot.yaml`
* Motor drive: `scripts/px4_motor_stream.py` (not `hwci run` throttle)

Flight Stand free-run / thrust tests are **SETUP A**:

* Rig: `rig.yaml` or `config/rig.flightstand.yaml`
* Profiles: `noprop_smoke*`, `efficiency_sweep`, etc.
* Command: `hwci run --config … --profile noprop_…`
