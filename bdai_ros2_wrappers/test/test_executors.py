# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.
import threading
from typing import Generator

import pytest
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.context import Context
from rclpy.node import Node
from std_srvs.srv import Trigger

from bdai_ros2_wrappers.executors import AutoScalingMultiThreadedExecutor, AutoScalingThreadPool


@pytest.fixture
def ros_context() -> Generator[Context, None, None]:
    context = Context()
    rclpy.init(context=context)
    try:
        yield context
    finally:
        context.try_shutdown()


@pytest.fixture
def ros_node(ros_context: Context) -> Generator[Node, None, None]:
    node = rclpy.create_node("pytest_node", context=ros_context)
    try:
        yield node
    finally:
        node.destroy_node()


def test_autoscaling_thread_pool() -> None:
    """
    Asserts that the autoscaling thread pool scales and de-scales on demand.
    """

    pool = AutoScalingThreadPool(max_idle_time=2.0)
    assert len(pool.workers) == 0
    assert not pool.working

    events = [threading.Event() for _ in range(10)]
    results = pool.map(lambda i, e: e.wait() and i, range(10), events)
    assert len(pool.workers) == len(events)
    assert pool.working
    assert not pool.capped, pool._waitqueues

    for e in events:
        e.set()
    assert list(results) == list(range(10))

    for worker in pool.workers:
        worker.join(timeout=10)
        assert not worker.is_alive()

    assert len(pool.workers) == 0
    assert not pool.working


def test_autoscaling_thread_pool_with_quota() -> None:
    """
    Asserts that the autoscaling thread pool respects submission quotas.
    """

    pool = AutoScalingThreadPool(submission_quota=5, submission_patience=1.0, max_idle_time=2.0)
    assert len(pool.workers) == 0
    assert not pool.working

    events = [threading.Event() for _ in range(10)]
    results = pool.map(lambda i, e: e.wait() and i, range(10), events)
    assert len(pool.workers) == 5
    assert pool.working
    assert pool.capped

    for e in events[:5]:
        e.set()
    assert len(pool.workers) == 5

    for e in events[5:]:
        e.set()
    assert len(pool.workers) == 5

    assert list(results) == list(range(10))
    assert len(pool.workers) == 5

    for worker in pool.workers:
        worker.join(timeout=10)
        assert not worker.is_alive()
    assert len(pool.workers) == 0
    assert not pool.working


def test_autoscaling_executor(ros_context: Context, ros_node: Node) -> None:
    """
    Asserts that the autoscaling multithreaded executor scales to attend a synchronous service call from a
    "one-shot" timer callback, serviced by the same executor.
    """

    def dummy_server_callback(_: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        response.success = True
        return response

    ros_node.create_service(
        Trigger, "/dummy/trigger", dummy_server_callback, callback_group=MutuallyExclusiveCallbackGroup()
    )

    client = ros_node.create_client(Trigger, "/dummy/trigger", callback_group=MutuallyExclusiveCallbackGroup())

    executor = AutoScalingMultiThreadedExecutor(context=ros_context)
    assert len(executor.thread_pool.workers) == 0
    assert not executor.thread_pool.working
    executor.add_node(ros_node)
    try:
        future = executor.create_task(lambda: client.call(Trigger.Request()))
        executor.spin_until_future_complete(future, timeout_sec=5)
        response = future.result()
        assert response.success
        assert executor.thread_pool.wait(timeout=10)
        assert not executor.thread_pool.working
    finally:
        executor.remove_node(ros_node)
        executor.shutdown()
