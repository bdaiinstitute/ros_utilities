# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.
import threading

import pytest
import rclpy.task
import std_srvs.srv
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from bdai_ros2_wrappers.executors import AutoScalingMultiThreadedExecutor, AutoScalingThreadPool


@pytest.fixture
def pytest_context():
    context = rclpy.context.Context()
    rclpy.init(context=context)
    try:
        yield context
    finally:
        context.try_shutdown()


@pytest.fixture
def pytest_node(pytest_context):
    node = rclpy.create_node(
        'pytest_node', context=pytest_context)
    try:
        yield node
    finally:
        node.destroy_node()


def test_autoscaling_thread_pool():
    """
    Asserts that the autoscaling thread pool scales and de-scales on demand.
    """

    pool = AutoScalingThreadPool(max_idle_time=2)
    assert len(pool.workers) == 0

    events = [threading.Event() for _ in range(10)]
    results = pool.map(lambda i, e: e.wait() and i, range(10), events)
    assert len(pool.workers) == len(events)

    for e in events:
        e.set()
    assert list(results) == list(range(10))

    for worker in pool.workers:
        worker.join(timeout=10)
        assert not worker.is_alive()

    assert len(pool.workers) == 0


def test_autoscaling_executor(pytest_context, pytest_node):
    """
    Asserts that the autoscaling multithreaded executor scales to
    attend a synchronous service call from a "one-shot" timer callback,
    serviced by the same executor.
    """
    def dummy_server_callback(_, response):
        response.success = True
        return response

    pytest_node.create_service(
        std_srvs.srv.Trigger, '/dummy/trigger', dummy_server_callback,
        callback_group=MutuallyExclusiveCallbackGroup())

    client = pytest_node.create_client(
        std_srvs.srv.Trigger, '/dummy/trigger',
        callback_group=MutuallyExclusiveCallbackGroup())

    future = rclpy.task.Future()

    def deferred_dummy_trigger():
        nonlocal timer
        timer.cancel()
        future.set_result(client.call(
            std_srvs.srv.Trigger.Request()))

    timer = pytest_node.create_timer(
        0.5, deferred_dummy_trigger,
        callback_group=MutuallyExclusiveCallbackGroup())

    executor = AutoScalingMultiThreadedExecutor(context=pytest_context)
    assert len(executor.thread_pool.workers) == 0
    executor.add_node(pytest_node)
    executor.spin_until_future_complete(future, timeout_sec=5)
    executor.remove_node(pytest_node)
    executor.shutdown()

    response = future.result()
    assert response.success
