# Copyright (c) 2024 Boston Dynamics AI Institute Inc.  All rights reserved.

from rclpy.task import Future

from bdai_ros2_wrappers.futures import wait_for_future
from bdai_ros2_wrappers.scope import ROSAwareScope


def test_wait_for_cancelled_future(ros: ROSAwareScope) -> None:
    """Asserts that waiting for a cancelled future does not hang indefinitely."""
    future = Future()
    future.cancel()

    assert not wait_for_future(future, context=ros.context)
    assert future.cancelled()
