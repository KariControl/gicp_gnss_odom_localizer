# Contributing

Contributions should preserve explicit frame, timestamp, observation-point, and covariance semantics.

## Development workflow

1. Create a focused branch and describe the sensor assumptions and ROS distribution used.
2. Run `./tools/run_reference_tests.sh` and `python3 tools/check_repository.py`.
3. Build and test the complete workspace in ROS 2 Jazzy.
4. For estimator changes, attach before/after rosbag metrics and the exact parameter files.
5. Do not enable an experimental path in public defaults without a repeatable regression test.

## Required evidence for algorithm changes

Report at least trajectory length, endpoint drift, relative-pose error where ground truth exists, rejected scan count, GNSS update/rejection count, outage duration, maximum recovery correction per update, and discontinuity at GNSS return. Include a corridor/low-observability sequence and a GNSS outage/reacquisition sequence.

## Coding rules

- C++17, finite-value checks at sensor boundaries, monotonic timestamp handling, and deterministic failure behavior.
- Do not infer heading validity from a quaternion value.
- Do not relabel an antenna observation as `base_link` without a valid lever arm and yaw.
- Do not silently substitute reception time, identity TF, point ordering, or constant speed.
- New factors require unit tests for normal operation, outliers, non-finite inputs, and rollback.
- Keep public YAML conservative; experimental features must be opt-in and documented.

## Pull requests

Explain the problem, algorithm, changed interfaces, safety/failure behavior, tests, and bag results. Update the changelog and migration guide for user-visible changes.
