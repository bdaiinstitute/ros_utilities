# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.
import logging
from typing import Generator, Optional

import pytest
import rclpy
from rcl_interfaces.msg import Log
from rclpy.context import Context
from rclpy.executors import Executor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future

from bdai_ros2_wrappers.logging import logs_to_ros


@pytest.fixture(autouse=True)
def ros_context() -> Generator[Context, None, None]:
    context = Context()
    args = ["--ros-args", "--enable-rosout-logs"]
    rclpy.init(context=context, args=args)
    try:
        yield context
    finally:
        context.try_shutdown()


@pytest.fixture
def ros_node(ros_context: Context) -> Generator[Node, None, None]:
    node = rclpy.create_node("test_node", context=ros_context)
    try:
        yield node
    finally:
        node.destroy_node()


@pytest.fixture
def ros_executor(ros_context: Context, ros_node: Node) -> Generator[Executor, None, None]:
    executor = SingleThreadedExecutor(context=ros_context)
    executor.add_node(ros_node)
    try:
        yield executor
    finally:
        executor.remove_node(ros_node)
        executor.shutdown()


def test_log_forwarding(ros_node: Node, ros_executor: Executor) -> None:
    future: Optional[Future] = None

    def callback(message: Log) -> None:
        nonlocal future
        if future and not future.done():
            future.set_result(message)

    ros_node.create_subscription(Log, "/rosout", callback, 10)

    future = Future()
    with logs_to_ros(ros_node):
        logger = logging.getLogger("my_logger")
        logger.propagate = True  # ensure propagation is enabled
        logger.info("test")

    ros_executor.spin_until_future_complete(future, timeout_sec=10)
    assert future.done()
    log = future.result()
    # NOTE(hidmic) why are log levels of bytestring type !?
    assert log.level == int.from_bytes(Log.INFO, byteorder="little")
    assert log.name == ros_node.get_logger().name
    assert log.msg == "(logging.my_logger) test"
