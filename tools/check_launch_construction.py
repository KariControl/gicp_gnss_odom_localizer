#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Construct project launch descriptions without requiring a ROS installation."""
from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path

sys.dont_write_bytecode = True

ROOT = Path(__file__).resolve().parents[1]


class Dummy:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs


class LaunchDescription(Dummy):
    pass


def install_stub_modules() -> None:
    def module(name: str) -> types.ModuleType:
        result = types.ModuleType(name)
        sys.modules[name] = result
        return result

    module("ament_index_python")
    packages = module("ament_index_python.packages")
    packages.get_package_share_directory = lambda name: f"/tmp/fake_share/{name}"

    launch = module("launch")
    launch.LaunchDescription = LaunchDescription
    actions = module("launch.actions")
    actions.DeclareLaunchArgument = Dummy
    actions.IncludeLaunchDescription = Dummy
    conditions = module("launch.conditions")
    conditions.IfCondition = Dummy
    conditions.UnlessCondition = Dummy
    substitutions = module("launch.substitutions")
    substitutions.LaunchConfiguration = Dummy
    substitutions.PythonExpression = Dummy
    sources = module("launch.launch_description_sources")
    sources.AnyLaunchDescriptionSource = Dummy
    sources.PythonLaunchDescriptionSource = Dummy

    module("launch_ros")
    ros_actions = module("launch_ros.actions")
    ros_actions.Node = Dummy
    ros_actions.ComposableNodeContainer = Dummy
    descriptions = module("launch_ros.descriptions")
    descriptions.ComposableNode = Dummy


def main() -> int:
    install_stub_modules()
    launch_directory = ROOT / "src/pure_odometry_bringup/launch"
    paths = (
        launch_directory / "odometry_standalone.launch.py",
        launch_directory / "odometry_container.launch.py",
        launch_directory / "autoware_lsim_localization.launch.py",
    )
    for index, path in enumerate(paths):
        spec = importlib.util.spec_from_file_location(f"launch_under_test_{index}", path)
        if spec is None or spec.loader is None:
            raise RuntimeError(f"could not load launch file: {path}")
        loaded = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(loaded)
        description = loaded.generate_launch_description()
        if not isinstance(description, LaunchDescription):
            raise RuntimeError(f"unexpected launch result: {path}")
        print(f"PASS launch construction {path.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
