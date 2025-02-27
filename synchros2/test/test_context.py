# Copyright (c) 2023 Boston Dynamics AI Institute LLC.  All rights reserved.

import rclpy

from synchros2.context import wait_for_shutdown
from synchros2.process import ROSAwareScope


def test_wait_for_shutdown(ros: ROSAwareScope) -> None:
    """Asserts that wait for shutdown works as expected."""
    assert not wait_for_shutdown(timeout_sec=1.0)
    assert ros.executor is not None
    ros.executor.create_task(lambda: rclpy.shutdown())
    assert wait_for_shutdown(timeout_sec=10.0)
