# Copyright (c) 2024 Robotics and AI Institute LLC dba RAI Institute.  All rights reserved.

import bdai_ros2_wrappers.scope
import synchros2.scope


def test_submodule_aliasing() -> None:
    assert id(bdai_ros2_wrappers.scope) == id(synchros2.scope)


def test_global_aliasing() -> None:
    with bdai_ros2_wrappers.scope.top(global_=True) as top:
        assert synchros2.scope.current() is top
