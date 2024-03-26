# Copyright (c) 2024 Boston Dynamics AI Institute Inc.  All rights reserved.

import threading
from typing import Generator

import pytest
import rclpy
from rclpy.context import Context
from std_srvs.srv import Trigger

from bdai_ros2_wrappers.executors import AutoScalingMultiThreadedExecutor
from bdai_ros2_wrappers.node import Node


@pytest.fixture
def ros_context() -> Generator[Context, None, None]:
    """A fixture yielding a managed rclpy.context.Context instance."""
    context = Context()
    rclpy.init(context=context)
    try:
        yield context
    finally:
        context.try_shutdown()


def test_node_destruction_during_execution(ros_context: Context) -> None:
    """Asserts that node destructionthe autoscaling multithreaded executor scales to attend a
    synchronous service call from a "one-shot" timer callback, serviced by
    the same executor.
    """

    def dummy_server_callback(_: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        response.success = True
        return response

    node = Node("pytest_node", context=ros_context)
    node.create_service(Trigger, "/dummy/trigger", dummy_server_callback)
    client = node.create_client(Trigger, "/dummy/trigger")

    executor = AutoScalingMultiThreadedExecutor(max_threads=1, context=ros_context)
    executor.add_node(node)

    barrier = threading.Barrier(2)
    try:
        # First smoke test the executor with a service invocation
        future = client.call_async(Trigger.Request())
        executor.spin_until_future_complete(future, timeout_sec=5.0)
        assert future.done() and future.result().success
        # Then block its sole worker thread
        executor.create_task(lambda: barrier.wait())
        executor.spin_once()
        # Then queue node destruction
        executor.create_task(lambda: node.destroy_node())
        executor.spin_once()
        assert not node.destruction_requested  # still queued
        # Then queue another service invocation
        future = client.call_async(Trigger.Request())
        executor.spin_once()
        # Unblock worker thread in executor
        barrier.wait()
        # Check that executor wraps up early due to node destruction
        executor.spin_until_future_complete(future, timeout_sec=5.0)
        assert node.destruction_requested
        assert executor.thread_pool.wait(timeout=5.0)
        assert not future.done()  # future response will never be resolved
    finally:
        barrier.reset()
        executor.remove_node(node)
        executor.shutdown()
