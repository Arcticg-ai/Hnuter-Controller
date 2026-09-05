# Repository consolidation record

## Source snapshots

The folder migration combines these previously divergent histories:

- `origin/main`: `c80eb6c794b30a410d137cca8d4b4bfea6700be4`
- `origin/hardware`: `754384cbf82fab48422317ad89046ea9c400efba`
- former SITL branch: `dc11ee45a36754bb416beadacd5bcc405cf9ed97`
- uncommitted hardware source patch captured from local commit `abeb52c`:
  SHA-256 `c454db96a9de3d73a469e8e5f839c497dc6a99eeefcbe4f314288febf7d71d8b`

The merge commit has both `main` and `hardware` histories as parents. The
following tags retain the exact branch tips before consolidation:

- `archive/main-pre-consolidation-20260830`
- `archive/hardware-pre-consolidation-20260830`
- `archive/sitl-pre-consolidation-20260830`

## Destination layout

- Shared algorithms and log paths: `controllers/common/`
- PX4/Gazebo and research entry points: `controllers/simulation/`
- Real-aircraft entry points: `controllers/hardware/`
- Tuning: `config/simulation/` and `config/hardware/`
- Tests: `tests/common/`, `tests/simulation/`, and `tests/hardware/`
- Experiment, plotting, and tuning utilities: `tools/`

The obsolete `hnuter_external_controller.py` is not restored because
`controllers/simulation/hnuter_external_controller_px4_position.py` is its
maintained replacement. Its exact content remains reachable through the
hardware archive tag and merge history.

## Reconciled changes

- The latest hardware Position, Direct, DRCDA, OK, and IEBC entry points were
  imported from `origin/hardware`.
- Local uncommitted DRCDA tail-model, handover, configuration, and test changes
  were applied with a three-way merge on top of the remote hardware tip.
- The remote IEBC braking/contact state machine was retained.
- The simulation IEBC core was synchronized byte-for-byte with the hardware
  `InteractionEnergyBarrierFilter`, fixing a real simulation/hardware drift.
- The latest web telemetry improvements were applied to the already organized
  `tools/tuning/` implementation without dropping its velocity, angular-rate,
  timeout, logging, or parameter-control features.

## Preserved outside the commit

The original dirty `/home/hnuter/px4_ws_ros2` hardware worktree remains
untouched. Its untracked flight records, raw ULog, and 2.46 GB MP4 remain at
their original paths and are deliberately excluded from this source migration.
They must not be published without separate payload-level approval.

## Runtime convention

Run controllers from the repository root as modules, for example:

```bash
python3 -m controllers.simulation.hnuter_external_direct_drcda
python3 -m controllers.hardware.hnuter_external_direct_drcda_hardware
```

This keeps imports stable after relocation. Logs still default to the
repository-level `hnuter_logs/` directory.
