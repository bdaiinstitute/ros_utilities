#  Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

from typing import Iterable

import pytest

import bdai_ros2_wrappers.scope as scope
from bdai_ros2_wrappers.scope import ROSAwareScope


@pytest.fixture(scope="function")
def ros() -> Iterable[ROSAwareScope]:
    with scope.top(global_=True, namespace="fixture") as top:
        yield top


@pytest.fixture(scope="function")
def verbose_ros() -> Iterable[ROSAwareScope]:
    args = ["--ros-args", "--enable-rosout-logs", "--log-level", "INFO"]
    with scope.top(args, global_=True, namespace="fixture") as top:
        yield top
