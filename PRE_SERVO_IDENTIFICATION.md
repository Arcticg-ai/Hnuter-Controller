# Pre-servo-identification controller snapshot

This branch preserves the complete external Hnuter controllers without an
identified servo plant predictor. Direct mode maps requested physical angles
straight to the configured primary and secondary actuator ranges and allocates
motor thrust from the requested geometry.

The gamepad hotplug handling, DDS localhost isolation, startup state machine,
conservative gain transition, and attitude-integral anti-windup remain present.
