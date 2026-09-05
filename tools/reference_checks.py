#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Small numerical reference checks independent of ROS."""
from __future__ import annotations

import math


def bounded_step(error: float, per_update: float, per_second: float, dt: float) -> float:
    limit = min(per_update, per_second * max(0.0, dt))
    return max(-limit, min(limit, error))


def lever_arm_covariance(var_x: float, var_y: float, var_yaw: float, rx: float, ry: float, yaw: float):
    c, s = math.cos(yaw), math.sin(yaw)
    jx = s * rx + c * ry
    jy = -c * rx + s * ry
    return (
        var_x + jx * jx * var_yaw,
        var_y + jy * jy * var_yaw,
        jx * jy * var_yaw,
    )


def main() -> None:
    assert math.isclose(bounded_step(10.0, 0.5, 1.0, 0.1), 0.1)
    assert math.isclose(bounded_step(-10.0, 0.5, 1.0, 2.0), -0.5)
    vx, vy, cxy = lever_arm_covariance(0.04, 0.04, 0.01, 2.0, 0.0, 0.0)
    assert math.isclose(vx, 0.04)
    assert math.isclose(vy, 0.08)
    assert math.isclose(cxy, 0.0)
    print("PASS reference_checks")


if __name__ == "__main__":
    main()
