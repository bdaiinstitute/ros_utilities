"""Provides a helper class for accessing the values of launch arguments from a LaunchConfiguration

Copyright (c) 2024 Boston Dynamics AI Institute LLC.  All rights reserved.
"""
# mypy: ignore-errors
from launch import LaunchContext
from launch.substitutions import LaunchConfiguration

from synchros2.launch.actions import convert_to_bool


class LaunchConfigurationValues:
    """Helper class for accessing the values of launch arguments from a LaunchConfiguration.

    This helper serves the following purposes:
    1. Avoid spending a line of code on each of a potentially large number of values retrieved from the launch config
    2. Enable easily accessing the values of boolean arguments as actual `bool` types when needed,
       while still normally treating them as `str` the way downstream launch operations usually expect.

    Example:
    ----------------------------------------------------------------
    def launch_setup(context: LaunchContext):
        launch_elements = []
        vals = LaunchConfigurationValues(context)
        vals["robot_name"]  # str
        vals.bool("use_afterburner")  # bool

        # The "condition" kwarg expects a launch operation on a string-type boolean, not an actual bool
        launch_elements.append(
            Node(
                [...]
                condition=IfCondition(vals["use_afterburner"]),  # Argument is in ["true", "false"], not [True, False]
            ),
        )

        if vals.bool("use_afterburner"):  # Access value as one of [True, False]
            # Add an additional launch element
            [...]
    ----------------------------------------------------------------
    """

    def __init__(self, context: LaunchContext):
        """Initialize a LaunchConfigurationValues helper for a given launch context

        Args:
            context : The LaunchContext being used in the launch_setup function where this helper class is needed.
        """
        self._context = context
        self._string_values: dict[str, str] = {}
        self._bool_values: dict[str, bool] = {}

    def __getitem__(self, launch_arg_name: str) -> str:
        """Retrieve the string value of the specified launch argument"""
        if launch_arg_name not in self._string_values:
            self._string_values[launch_arg_name] = LaunchConfiguration(launch_arg_name).perform(self._context)
        return self._string_values[launch_arg_name]

    def bool(self, launch_arg_name: str) -> bool:  # noqa: A003
        """Retrieve the boolean value of the specified launch argument.

        This method should fail if the argument's string value is not one of the expected options for a boolean.
        """
        if launch_arg_name not in self._bool_values:
            self._bool_values[launch_arg_name] = convert_to_bool(launch_arg_name, self.__getitem__(launch_arg_name))
        return self._bool_values[launch_arg_name]
