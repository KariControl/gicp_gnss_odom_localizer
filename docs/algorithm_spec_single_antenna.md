# Single-antenna GNSS algorithm specification

## Observation

A GGA measurement is the map-frame position of the antenna, not the vehicle origin. The converter publishes

- `p_map_antenna`;
- horizontal covariance derived from configured fix-quality/HDOP/correction-age models;
- optional base yaw and yaw covariance;
- `r_base_antenna` from calibrated TF or an explicitly enabled parameter source.

The fusion model is

`h(x, y, yaw) = [x, y] + R(yaw) r_base_antenna`.

Its yaw column is the derivative of the rotated lever arm, so yaw uncertainty and position/yaw cross-covariance affect the position update.

## Heading

Absolute heading priority is dual antenna, Doppler course, then a position trajectory. For the single-antenna trajectory, endpoints must pass confidence and baseline gates. Corrected IMU yaw can curve-correct that chord and can propagate an already valid absolute heading for a bounded duration. It cannot initialize absolute yaw alone.

When no heading is valid, the observation remains position-only. Initialization waits for enough vehicle motion to estimate map-to-odom yaw from corresponding GNSS and odom trajectories, or for enough independent heading samples.

## Outage return

After a GNSS outage, synchronized antenna/base-consistent positions and optional headings are accumulated. Robust weighted SE(2) alignment must pass temporal, observability, residual, and heading consistency gates. One isolated outlier may be removed, but a single remaining Fix is never sufficient.

The recovered map-to-odom target is approached with bounded translation and yaw rates. Several low-residual samples are required before returning to normal tracking.
