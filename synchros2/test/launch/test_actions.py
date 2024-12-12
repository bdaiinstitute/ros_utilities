# Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.
from enum import Enum

import pytest

from synchros2.launch.actions import DeclareBooleanLaunchArgument, DeclareEnumLaunchArgument


def test_declare_boolean_launch_argument_default_value_false() -> None:
    arg = DeclareBooleanLaunchArgument("arg", default_value="false")
    assert arg.default_value is not None
    assert len(arg.default_value) == 1
    assert arg.default_value[0].text == "false"


def test_declare_boolean_launch_argument_default_value_False() -> None:
    arg = DeclareBooleanLaunchArgument("arg", default_value="False")
    assert arg.default_value is not None
    assert len(arg.default_value) == 1
    assert arg.default_value[0].text == "False"


def test_declare_boolean_launch_argument_default_value_bool_false() -> None:
    arg = DeclareBooleanLaunchArgument("arg", default_value=False)
    assert arg.default_value is not None
    assert len(arg.default_value) == 1
    assert arg.default_value[0].text == "false"


def test_declare_boolean_launch_argument_default_value_true() -> None:
    arg = DeclareBooleanLaunchArgument("arg", default_value="true")
    assert arg.default_value is not None
    assert len(arg.default_value) == 1
    assert arg.default_value[0].text == "true"


def test_declare_boolean_launch_argument_default_value_True() -> None:
    arg = DeclareBooleanLaunchArgument("arg", default_value="True")
    assert arg.default_value is not None
    assert len(arg.default_value) == 1
    assert arg.default_value[0].text == "True"


def test_declare_boolean_launch_argument_default_value_bool_true() -> None:
    arg = DeclareBooleanLaunchArgument("arg", default_value=True)
    assert arg.default_value is not None
    assert len(arg.default_value) == 1
    assert arg.default_value[0].text == "true"


def test_declare_boolean_launch_argument_set_choices() -> None:
    with pytest.raises(KeyError):
        _ = DeclareBooleanLaunchArgument("arg", choices=["some", "choices"])


def test_declare_boolean_launch_argument_default_value_invalid() -> None:
    with pytest.raises(ValueError):
        _ = DeclareBooleanLaunchArgument("arg", default_value="not true or false")


class MockStrEnum(str, Enum):
    A = "A"
    B = "B"
    C = "C"


def test_declare_enum_launch_argument_str_enum_str_default_value() -> None:
    arg = DeclareEnumLaunchArgument(MockStrEnum, "arg", default_value="A")
    assert arg.default_value is not None
    assert len(arg.default_value) == 1
    assert arg.default_value[0].text == "A"


def test_declare_enum_launch_argument_str_enum_enum_default_value() -> None:
    arg = DeclareEnumLaunchArgument(MockStrEnum, "arg", default_value=MockStrEnum.A)
    assert arg.default_value is not None
    assert len(arg.default_value) == 1
    assert arg.default_value[0].text == "A"


def test_declare_enum_launch_argument_set_choices() -> None:
    with pytest.raises(KeyError):
        _ = DeclareEnumLaunchArgument(MockStrEnum, "arg", choices=["some", "choices"])


def test_declare_enum_launch_argument_invalid_default() -> None:
    with pytest.raises(ValueError):
        _ = DeclareEnumLaunchArgument(MockStrEnum, "arg", default_value="D")
