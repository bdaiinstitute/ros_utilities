# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.

import rclpy

from bdai_ros2_wrappers.context import wait_for_shutdown
from bdai_ros2_wrappers.process import ROSAwareScope


def test_wait_for_shutdown(ros: ROSAwareScope) -> None:
    """Asserts that wait for shutdown works as expected."""
    assert not wait_for_shutdown(timeout_sec=1.0)
    assert ros.executor is not None
    ros.executor.create_task(lambda: rclpy.shutdown())
    assert wait_for_shutdown(timeout_sec=10.0)
