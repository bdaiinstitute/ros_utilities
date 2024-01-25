# Copyright (c) 2024 Boston Dynamics AI Institute Inc.  All rights reserved.
import array
import collections
import time
from typing import Iterable
from unittest.mock import Mock

import pytest
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from example_interfaces.action import Fibonacci
from rclpy.action.server import ActionServer, CancelResponse, GoalResponse, ServerGoalHandle

from bdai_ros2_wrappers.action import Actionable, ActionAborted, ActionCancelled, ActionRejected
from bdai_ros2_wrappers.futures import wait_for_future
from bdai_ros2_wrappers.scope import ROSAwareScope


def fibonacci_sequence(order: int) -> Iterable[int]:
    """Generate a Fibonacci sequence for the given `order`."""
    yield 0
    if order > 0:
        yield 1
    if order > 1:
        sequence: collections.deque = collections.deque([0, 1], maxlen=2)
        for partial_order in range(2, order + 1):
            sequence.append(sequence[-1] + sequence[-2])
            yield sequence[-1]


def default_execute_callback(goal_handle: ServerGoalHandle) -> Fibonacci.Result:
    """Execute default Fibonacci action behavior."""
    feedback = Fibonacci.Feedback()

    for number in fibonacci_sequence(goal_handle.request.order):
        feedback.sequence.append(number)
        goal_handle.publish_feedback(feedback)
        time.sleep(0.01)

    goal_handle.succeed()

    result = Fibonacci.Result()
    result.sequence = feedback.sequence
    return result


def test_successful_synchronous_action_invocation(ros: ROSAwareScope) -> None:
    ActionServer(ros.node, Fibonacci, "fibonacci/compute", default_execute_callback)
    compute_fibonacci = Actionable(Fibonacci, "fibonacci/compute", ros.node)
    result = compute_fibonacci(Fibonacci.Goal(order=5), timeout_sec=10.0)
    expected_result = array.array("i", [0, 1, 1, 2, 3, 5])
    assert result.sequence == expected_result


def test_successful_synchronous_action_invocation_with_feedback(ros: ROSAwareScope) -> None:
    mock_callback = Mock()
    ActionServer(ros.node, Fibonacci, "fibonacci/compute", default_execute_callback)
    compute_fibonacci = Actionable(Fibonacci, "fibonacci/compute", ros.node)
    result = compute_fibonacci(Fibonacci.Goal(order=5), feedback_callback=mock_callback, timeout_sec=10.0)
    expected_result = array.array("i", [0, 1, 1, 2, 3, 5])
    assert result.sequence == expected_result
    assert mock_callback.call_count > 0


def test_successful_asynchronous_action_invocation(ros: ROSAwareScope) -> None:
    ActionServer(ros.node, Fibonacci, "fibonacci/compute", default_execute_callback)
    compute_fibonacci = Actionable(Fibonacci, "fibonacci/compute", ros.node)
    action = compute_fibonacci.asynchronously(Fibonacci.Goal(order=5))
    assert wait_for_future(action.finalization, timeout_sec=10.0)
    assert action.acknowledged
    assert action.accepted
    assert action.finalized
    assert not action.aborted
    assert not action.cancelled
    assert action.succeeded
    expected_result = array.array("i", [0, 1, 1, 2, 3, 5])
    assert action.result.sequence == expected_result


def test_successful_asynchronous_action_invocation_with_feedback(ros: ROSAwareScope) -> None:
    ActionServer(ros.node, Fibonacci, "fibonacci/compute", default_execute_callback)
    compute_fibonacci = Actionable(Fibonacci, "fibonacci/compute", ros.node)
    action = compute_fibonacci.asynchronously(Fibonacci.Goal(order=5), track_feedback=True)
    assert wait_for_future(action.acknowledgement, timeout_sec=10.0)
    streamed_sequences = []
    for order, feedback in enumerate(action.feedback_stream(timeout_sec=2.0)):
        assert feedback.sequence == array.array("i", fibonacci_sequence(order))
        streamed_sequences.append(feedback.sequence)
    assert action.finalized
    assert action.succeeded
    assert not action.aborted
    assert not action.cancelled
    expected_result = array.array("i", [0, 1, 1, 2, 3, 5])
    assert action.result.sequence == expected_result
    stored_sequences = [feedback.sequence for feedback in action.feedback]
    assert stored_sequences == streamed_sequences


def test_successful_asynchronous_action_invocation_with_limited_feedback(ros: ROSAwareScope) -> None:
    ActionServer(ros.node, Fibonacci, "fibonacci/compute", default_execute_callback)
    compute_fibonacci = Actionable(Fibonacci, "fibonacci/compute", ros.node)
    action = compute_fibonacci.asynchronously(Fibonacci.Goal(order=5), track_feedback=1)
    assert wait_for_future(action.acknowledgement, timeout_sec=10.0)
    *_, last_feedback = action.feedback_stream(buffer_size=10, timeout_sec=2.0)
    assert action.finalized
    assert action.succeeded
    assert not action.aborted
    assert not action.cancelled
    expected_result = array.array("i", [0, 1, 1, 2, 3, 5])
    assert action.result.sequence == expected_result
    assert len(action.feedback) == 1
    assert action.feedback[-1].sequence == last_feedback.sequence


def test_rejected_synchronous_action_invocation(ros: ROSAwareScope) -> None:
    ActionServer(
        ros.node, Fibonacci, "fibonacci/compute", default_execute_callback, goal_callback=lambda _: GoalResponse.REJECT
    )
    compute_fibonacci = Actionable(Fibonacci, "fibonacci/compute", ros.node)
    with pytest.raises(ActionRejected):
        compute_fibonacci(Fibonacci.Goal(order=5), timeout_sec=10.0)


def test_rejected_asynchronous_action_invocation(ros: ROSAwareScope) -> None:
    ActionServer(
        ros.node, Fibonacci, "fibonacci/compute", default_execute_callback, goal_callback=lambda _: GoalResponse.REJECT
    )
    compute_fibonacci = Actionable(Fibonacci, "fibonacci/compute", ros.node)
    action = compute_fibonacci.asynchronously(Fibonacci.Goal(order=5))
    assert wait_for_future(action.finalization, timeout_sec=10.0)
    assert action.acknowledged
    assert action.finalized
    assert not action.accepted
    assert not action.aborted
    assert not action.succeeded
    assert not action.cancelled


def test_aborted_synchronous_action_invocation(ros: ROSAwareScope) -> None:
    def execute_callback(goal_handle: ServerGoalHandle) -> Fibonacci.Result:
        goal_handle.abort()
        result = Fibonacci.Result()
        return result

    ActionServer(ros.node, Fibonacci, "fibonacci/compute", execute_callback)
    compute_fibonacci = Actionable(Fibonacci, "fibonacci/compute", ros.node)
    with pytest.raises(ActionAborted):
        compute_fibonacci(Fibonacci.Goal(order=5), timeout_sec=10.0)


def test_aborted_asynchronous_action_invocation(ros: ROSAwareScope) -> None:
    def execute_callback(goal_handle: ServerGoalHandle) -> Fibonacci.Result:
        goal_handle.abort()
        result = Fibonacci.Result()
        return result

    ActionServer(ros.node, Fibonacci, "fibonacci/compute", execute_callback)
    compute_fibonacci = Actionable(Fibonacci, "fibonacci/compute", ros.node)
    action = compute_fibonacci.asynchronously(Fibonacci.Goal(order=5))
    assert wait_for_future(action.finalization, timeout_sec=10.0)
    assert action.status == GoalStatus.STATUS_ABORTED
    assert action.acknowledged
    assert action.accepted
    assert action.finalized
    assert not action.cancelled
    assert action.aborted
    assert not action.succeeded


def test_cancelled_synchronous_action_invocation(ros: ROSAwareScope) -> None:
    def execute_callback(goal_handle: ServerGoalHandle) -> Fibonacci.Result:
        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Fibonacci.Result()
            time.sleep(0.01)

    ActionServer(
        ros.node, Fibonacci, "fibonacci/compute", execute_callback, cancel_callback=lambda _: CancelResponse.ACCEPT
    )
    compute_fibonacci = Actionable(Fibonacci, "fibonacci/compute", ros.node)

    assert ros.node is not None
    cancel_goal_client = ros.node.create_client(CancelGoal, "fibonacci/compute/_action/cancel_goal")

    def deferred_cancel() -> None:
        time.sleep(0.5)
        cancel_goal_client.call(CancelGoal.Request())

    assert ros.executor is not None
    ros.executor.create_task(deferred_cancel)
    with pytest.raises(ActionCancelled):
        compute_fibonacci(Fibonacci.Goal(order=5), timeout_sec=10.0)


def test_cancelled_asynchronous_action_invocation(ros: ROSAwareScope) -> None:
    def execute_callback(goal_handle: ServerGoalHandle) -> Fibonacci.Result:
        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Fibonacci.Result()
            time.sleep(0.01)

    ActionServer(
        ros.node, Fibonacci, "fibonacci/compute", execute_callback, cancel_callback=lambda _: CancelResponse.ACCEPT
    )
    compute_fibonacci = Actionable(Fibonacci, "fibonacci/compute", ros.node)
    action = compute_fibonacci.asynchronously(Fibonacci.Goal(order=5))
    assert wait_for_future(action.acknowledgement, timeout_sec=10.0)
    assert action.acknowledged
    assert action.accepted
    cancellation_future = action.cancel()
    assert wait_for_future(cancellation_future, timeout_sec=10.0)
    assert cancellation_future.result() == CancelGoal.Response.ERROR_NONE
    assert wait_for_future(action.finalization, timeout_sec=10.0)
    assert action.finalized
    assert action.status == GoalStatus.STATUS_CANCELED
    assert action.cancelled
    assert not action.aborted
    assert not action.succeeded
