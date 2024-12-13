# Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.
from __future__ import annotations

from typing import Literal

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from synchros2.launch.actions import DeclareBooleanLaunchArgument

_ROBOT_NAME: Literal["robot_name"] = "robot_name"
_VERBOSE: Literal["verbose"] = "verbose"


def add_robot_name_argument(ld: LaunchDescription) -> LaunchConfiguration:
    """Adds a launch argument for `robot_name` to the `LaunchDescription`

    Args:
        ld: The launch description instance

    Returns:
        A launch configuration that can be used to parse the command line argument for `robot_name`
    """
    ld.add_action(
        DeclareLaunchArgument(
            _ROBOT_NAME,
            description="Name of the robot.",
            default_value="",
        ),
    )
    return LaunchConfiguration(_ROBOT_NAME)


def add_verbose_argument(ld: LaunchDescription) -> LaunchConfiguration:
    """Adds a launch argument for `verbose` to the `LaunchDescription`

    Args:
        ld: The launch description instance

    Returns:
        A launch configuration that can be used to parse the command line argument for `verbose`
    """
    ld.add_action(DeclareBooleanLaunchArgument(_VERBOSE, description="Run with debug logging on.", default_value=False))
    return LaunchConfiguration(_VERBOSE)
