# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.

import argparse

import pytest

from bdai_ros2_wrappers.utilities import either_or, ensure, namespace_with


def test_either_or() -> None:
    assert either_or(None, "value", True)
    data = argparse.Namespace(value=True)
    assert either_or(data, "value", False)
    data = argparse.Namespace(value=True, getter=lambda obj: obj.value)
    assert either_or(data, "getter", False)


def test_namespace_with() -> None:
    assert namespace_with(None, "foo") == "foo"
    assert namespace_with("", "foo") == "foo"
    assert namespace_with("/", "foo") == "/foo"
    assert namespace_with("foo", "bar") == "foo/bar"


def test_ensure() -> None:
    data = None
    with pytest.raises(ValueError) as excinfo:
        ensure(data)
    assert "ensure(data) failed" in str(excinfo.value)
