# Precision evaluation tools

These repository-only tools validate recorded precision-localization runs and
generate publication evidence. They are intentionally not installed by
`pure_precision_bringup`; normal localization users therefore do not inherit
rosbag, NumPy, or Matplotlib runtime dependencies.

The scripts are:

- `validate_precision_bag.py`: validate exact-key and real-time contracts in a
  recorded precision run.
- `evaluate_accepted_scan_nonintrusion.py`: compare accepted-scan
  instrumentation between control and precision runs.
- `evaluate_startup_acceptance.py`: check startup publication and global-yaw
  safety gates.
- `evaluate_precision_glim_ab.py`: compare speed and isolated-precision runs
  against a common GLIM reference.
- `aggregate_precision_glim_ab.py`: pool multiple GLIM A/B result files.
- `validate_hesai_gnss_publication_run.py`: fail closed on dataset identity,
  runtime contracts, and repository/install/artifact SHA-256 provenance.

On ROS 2 Jazzy/Ubuntu 24.04, install the optional evaluation dependencies with:

```bash
sudo apt install \
  python3-matplotlib python3-numpy \
  ros-jazzy-rosbag2-py ros-jazzy-rosbag2-storage-mcap \
  ros-jazzy-rosidl-runtime-py
```

Run the dependency-light evaluator checks from the repository root:

```bash
python3 -B tools/evaluation/precision/test/test_precision_evaluation_helpers.py
python3 -B tools/evaluation/precision/test/test_hesai_gnss_publication_validator.py
python3 -B tools/evaluation/precision/scripts/validate_hesai_gnss_publication_run.py --self-test
```

Recording-specific configuration is installed separately by
`pure_localization_evaluation_profiles`, preserving the publication validator's
repository/install/artifact SHA-256 provenance chain.
