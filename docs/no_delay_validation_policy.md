# No-delay validation policy

All Hnuter external-controller simulation validation must use the no-delay PX4
and Gazebo plant. Delay-model branches and delayed/dynamic actuator plugins are
historical artifacts only and must not be used to rank, tune, or validate an
allocation algorithm.

The approved default firmware worktree is:

```text
/home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter-tail-sitl
```

The experiment runner enforces `validation_policy=no_delay_only` and rejects a
model containing known transport-delay or independent actuator-dynamics plugin
tokens. There is no command-line override.

Permitted physical constraints include servo angle limits, thrust limits,
direction-dependent static gain, joint PID behavior already present in the
approved plant, and actuator command-rate limits. These constraints do not
constitute a delay model.

Historical delay code and branches remain archived for provenance. Their data
must not be mixed into current comparison plots, aggregate metrics, tuning
decisions, or claims of performance improvement.
