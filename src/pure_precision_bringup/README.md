# Pure precision bringup

`pure_precision_bringup` starts the optional isolated scan-to-submap branch
beside the primary scan-to-scan/GNSS localization stack.

The base odometer must load
`config/submap_snapshot_override.yaml`. This keeps
`lidar_odom.tracking_mode=scan_to_scan` and enables accepted-scan snapshots
for the separate matcher. Neither precision node publishes TF or feeds a
correction back into the primary odometer or GNSS fusion.

## Outputs

- `/localization/precision_local_odom`
- `/localization/precision_global_odom`
- `/localization/precision_global_pose`

The existing `/localization/gyro_lidar_odom` and
`/localization/ekf_odom` outputs remain unchanged.

## Launch

Start the primary stack with the snapshot override, then launch the overlay:

```bash
ros2 launch pure_precision_bringup precision_overlay.launch.py \
  use_sim_time:=false
```

The default global branch accepts anchor updates only while typed fusion
authority is fresh and fully healthy. Loss of authority freezes its anchor;
recovery requires stable new candidates and applies bounded corrections.

For Docker replay, `run_autoware_lsim_docker.sh --tracking-mode
scan_to_submap` enables the same overlay. To validate a recorded run:

```bash
python3 tools/evaluation/precision/scripts/validate_precision_bag.py \
  /path/to/localization_output \
  --expected-rate 1.0
```

See [Architecture](../../docs/architecture.md) for ownership and failure
isolation, and
[the validator guide](../../tools/evaluation/precision/README.md) for optional
dependencies.
