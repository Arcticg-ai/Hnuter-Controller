# DRCDA pre-optimization baseline

The repository consolidation commit and tag
`archive/drcda-pre-optimization-20260906` freeze the complete DRCDA baseline
before any new optimization work.

Baseline implementation files:

- `controllers/common/hnuter_drcda.py`
- `controllers/simulation/hnuter_external_direct_drcda.py`
- `controllers/hardware/hnuter_external_direct_drcda_hardware.py`
- `config/simulation/no_delay_drcda_tuning.json`
- `config/hardware/hnuter_drcda_hardware_tuning.json`
- `tests/common/test_hnuter_drcda.py`
- `tests/hardware/test_hnuter_external_direct_drcda_hardware.py`
- `tools/experiments/run_no_delay_maneuver_experiments.py`
- `tools/experiments/analyze_no_delay_maneuver_experiments.py`
- `tools/plotting/plot_drcda_ablation.py`

The baseline must remain unchanged during the next algorithm iteration. New
solver/controller code belongs under `controllers/experiments/drcda_v2/`, and
new tuning/results belong under a separately named experiment directory.

Evidence labels remain strict: automated tests prove software behavior; SITL
results prove only the selected Gazebo plant and scenario; neither is a real
flight validation of actuator delay prediction.
