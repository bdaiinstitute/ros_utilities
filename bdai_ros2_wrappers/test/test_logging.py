# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.
import logging
from typing import Optional

from rcl_interfaces.msg import Log
from rclpy.task import Future

from bdai_ros2_wrappers.futures import wait_for_future
from bdai_ros2_wrappers.logging import logs_to_ros
from bdai_ros2_wrappers.scope import ROSAwareScope


def test_log_forwarding(verbose_ros: ROSAwareScope) -> None:
    future: Optional[Future] = None

    def callback(message: Log) -> None:
        nonlocal future
        if future and not future.done():
            future.set_result(message)

    assert verbose_ros.node is not None
    verbose_ros.node.create_subscription(Log, "/rosout", callback, 10)

    future = Future()
    with logs_to_ros(verbose_ros.node):
        logger = logging.getLogger("my_logger")
        logger.propagate = True  # ensure propagation is enabled
        logger.info("test")

    assert wait_for_future(future, timeout_sec=10)
    assert future.done()
    log = future.result()
    # NOTE(hidmic) why are log levels of bytestring type !?
    assert log.level == int.from_bytes(Log.INFO, byteorder="little")
    assert log.name == verbose_ros.node.get_logger().name
    assert log.msg == "(logging.my_logger) test"
