#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Small numerical reference checks independent of ROS."""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    yaw: float


def wrap_yaw(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def compose(left: Pose2, right: Pose2) -> Pose2:
    c = math.cos(left.yaw)
    s = math.sin(left.yaw)
    return Pose2(
        left.x + c * right.x - s * right.y,
        left.y + s * right.x + c * right.y,
        wrap_yaw(left.yaw + right.yaw),
    )


def inverse(pose: Pose2) -> Pose2:
    c = math.cos(pose.yaw)
    s = math.sin(pose.yaw)
    return Pose2(
        -c * pose.x - s * pose.y,
        s * pose.x - c * pose.y,
        wrap_yaw(-pose.yaw),
    )


def assert_pose_close(actual: Pose2, expected: Pose2, tolerance: float = 1.0e-10) -> None:
    assert math.isclose(actual.x, expected.x, abs_tol=tolerance)
    assert math.isclose(actual.y, expected.y, abs_tol=tolerance)
    assert math.isclose(wrap_yaw(actual.yaw - expected.yaw), 0.0, abs_tol=tolerance)


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


def check_submap_anchor_alignment() -> None:
    odom_anchor = Pose2(5.0, -2.0, 0.3)
    anchor_base = Pose2(4.0, 1.0, -0.2)
    optimized_odom_base = Pose2(9.2, 0.4, 0.12)

    realigned_odom_anchor = compose(optimized_odom_base, inverse(anchor_base))
    assert_pose_close(compose(realigned_odom_anchor, anchor_base), optimized_odom_base)

    # Reanchoring the submap on an existing keyframe must preserve every pose in odom.
    old_anchor_keyframes = [
        Pose2(2.0, -0.5, 0.1),
        Pose2(5.0, 0.2, 0.15),
        Pose2(7.0, 1.0, 0.05),
    ]
    old_odom_poses = [compose(odom_anchor, pose) for pose in old_anchor_keyframes]
    old_anchor_new_anchor = old_anchor_keyframes[0]
    new_anchor_old_anchor = inverse(old_anchor_new_anchor)
    new_odom_anchor = compose(odom_anchor, old_anchor_new_anchor)
    new_anchor_keyframes = [
        compose(new_anchor_old_anchor, pose) for pose in old_anchor_keyframes
    ]
    for expected, rebased in zip(old_odom_poses, new_anchor_keyframes, strict=True):
        assert_pose_close(compose(new_odom_anchor, rebased), expected)


def main() -> None:
    assert math.isclose(bounded_step(10.0, 0.5, 1.0, 0.1), 0.1)
    assert math.isclose(bounded_step(-10.0, 0.5, 1.0, 2.0), -0.5)
    vx, vy, cxy = lever_arm_covariance(0.04, 0.04, 0.01, 2.0, 0.0, 0.0)
    assert math.isclose(vx, 0.04)
    assert math.isclose(vy, 0.08)
    assert math.isclose(cxy, 0.0)
    check_submap_anchor_alignment()
    print("PASS reference_checks")


if __name__ == "__main__":
    main()
