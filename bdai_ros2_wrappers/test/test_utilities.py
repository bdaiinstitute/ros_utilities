# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.

import argparse

from bdai_ros2_wrappers.utilities import either_or


def test_either_or() -> None:
    assert either_or(None, "value", True)
    data = argparse.Namespace(value=True)
    assert either_or(data, "value", False)
    data = argparse.Namespace(value=True, getter=lambda obj: obj.value)
    assert either_or(data, "getter", False)
