# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.

import argparse

from bdai_ros2_wrappers.utilities import either_or, namespace


def test_either_or() -> None:
    assert either_or(None, "value", True)
    data = argparse.Namespace(value=True)
    assert either_or(data, "value", False)
    data = argparse.Namespace(value=True, getter=lambda obj: obj.value)
    assert either_or(data, "getter", False)


def test_namespace() -> None:
    assert namespace(None, "foo") == "foo"
    assert namespace("", "foo") == "foo"
    assert namespace("/", "foo") == "/foo"
    assert namespace("foo", "bar") == "foo/bar"
