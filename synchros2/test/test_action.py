# Copyright (c) 2024 Boston Dynamics AI Institute LLC.  All rights reserved.
import array
import threading
import time
from collections import deque
from typing import Iterable
from unittest.mock import Mock

import pytest
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from example_interfaces.action import Fibonacci
from rclpy.action.server import ActionServer, CancelResponse, GoalResponse, ServerGoalHandle
from rclpy.executors import SingleThreadedExecutor
from typing_extensions import TypeAlias

import synchros2.scope as ros_scope
from synchros2.action import Actionable, ActionAborted, ActionCancelled, ActionRejected
from synchros2.executors import foreground
from synchros2.futures import wait_for_future
from synchros2.node import Node
from synchros2.scope import ROSAwareScope


def fibonacci_sequence(order: int) -> Iterable[int]:
    """Generate a Fibonacci sequence for the given `order`."""
    yield 0
    if order > 0:
        yield 1
    if order > 1:
        sequence: deque = deque([0, 1], maxlen=2)
        for _partial_order in range(2, order + 1):
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


FibonacciActionable: TypeAlias = Actionable[Fibonacci.Goal, Fibonacci.Result, Fibonacci.Feedback]


def test_successful_synchronous_action_invocation(ros: ROSAwareScope) -> None:
    ActionServer(ros.node, Fibonacci, "fibonacci/compute", default_execute_callback)
    compute_fibonacci: FibonacciActionable = Actionable(Fibonacci, "fibonacci/compute", ros.node)
    assert compute_fibonacci.wait_for_server(timeout_sec=2.0)
    result = compute_fibonacci(Fibonacci.Goal(order=5), timeout_sec=10.0)
    expected_result = array.array("i", [0, 1, 1, 2, 3, 5])
    assert result.sequence == expected_result


def test_successful_synchronous_composed_action_invocation(ros: ROSAwareScope) -> None:
    ActionServer(ros.node, Fibonacci, "fibonacci/compute", default_execute_callback)
    compute_fibonacci: FibonacciActionable = Actionable(Fibonacci, "fibonacci/compute", ros.node)
    compute_fibonacci_sequences = compute_fibonacci.vectorized.compose(
        lambda n: (Fibonacci.Goal(order=i) for i in range(n + 1)),
    )
    assert compute_fibonacci_sequences.wait_for_server(timeout_sec=2.0)
    results = compute_fibonacci_sequences.synchronously(5, timeout_sec=5.0)
    expected_results = [
        array.array("i", [0]),
        array.array("i", [0, 1]),
        array.array("i", [0, 1, 1]),
        array.array("i", [0, 1, 1, 2]),
        array.array("i", [0, 1, 1, 2, 3]),
        array.array("i", [0, 1, 1, 2, 3, 5]),
    ]
    assert [result.sequence for result in results] == expected_results


def test_successful_synchronous_action_invocation_with_feedback(ros: ROSAwareScope) -> None:
    mock_callback = Mock()
    ActionServer(ros.node, Fibonacci, "fibonacci/compute", default_execute_callback)
    compute_fibonacci: FibonacciActionable = Actionable(Fibonacci, "fibonacci/compute", ros.node)
    assert compute_fibonacci.wait_for_server(timeout_sec=2.0)
    result = compute_fibonacci(Fibonacci.Goal(order=5), feedback_callback=mock_callback, timeout_sec=10.0)
    expected_result = array.array("i", [0, 1, 1, 2, 3, 5])
    assert result.sequence == expected_result
    assert mock_callback.call_count > 0


def test_successful_asynchronous_action_invocation(ros: ROSAwareScope) -> None:
    ActionServer(ros.node, Fibonacci, "fibonacci/compute", default_execute_callback)
    compute_fibonacci: FibonacciActionable = Actionable(Fibonacci, "fibonacci/compute", ros.node)
    assert compute_fibonacci.wait_for_server(timeout_sec=2.0)
    action = compute_fibonacci.asynchronously(Fibonacci.Goal(order=5))
    assert wait_for_future(action.finalization, timeout_sec=10.0)
    assert action.acknowledged
    assert action.accepted
    assert action.finalized
    assert not action.aborted
    assert not action.cancelled
    assert action.succeeded
    assert action.status == GoalStatus.STATUS_SUCCEEDED
    expected_result = array.array("i", [0, 1, 1, 2, 3, 5])
    assert action.result.sequence == expected_result


def test_successful_asynchronous_composed_action_invocation(ros: ROSAwareScope) -> None:
    ActionServer(ros.node, Fibonacci, "fibonacci/compute", default_execute_callback)
    compute_fibonacci: FibonacciActionable = Actionable(Fibonacci, "fibonacci/compute", ros.node)
    compute_fibonacci_sequences = compute_fibonacci.vectorized.compose(
        lambda n: (Fibonacci.Goal(order=i) for i in range(n + 1)),
    )
    assert compute_fibonacci_sequences.wait_for_server(timeout_sec=2.0)
    future = compute_fibonacci_sequences.asynchronously(5)
    assert wait_for_future(future, timeout_sec=10.0)
    expected_results = [
        array.array("i", [0]),
        array.array("i", [0, 1]),
        array.array("i", [0, 1, 1]),
        array.array("i", [0, 1, 1, 2]),
        array.array("i", [0, 1, 1, 2, 3]),
        array.array("i", [0, 1, 1, 2, 3, 5]),
    ]
    assert [result.sequence for result in future.result()] == expected_results


def test_spin_on_succesful_asynchronous_action_invocation() -> None:
    with ros_scope.top(global_=True, prebaked=False, namespace="fixture") as ros, foreground(
        SingleThreadedExecutor(),
    ) as ros.executor, ros.managed(Node, node_name="test_node") as ros.node:
        ActionServer(ros.node, Fibonacci, "fibonacci/compute", default_execute_callback)
        compute_fibonacci: FibonacciActionable = Actionable(Fibonacci, "fibonacci/compute", ros.node)
        assert compute_fibonacci.wait_for_server(timeout_sec=2.0)
        action = compute_fibonacci.asynchronously(Fibonacci.Goal(order=5))
        ros.executor.spin_until_future_complete(action.as_future(), timeout_sec=10.0)
        assert action.status == GoalStatus.STATUS_SUCCEEDED


def test_successful_asynchronous_action_invocation_with_feedback(ros: ROSAwareScope) -> None:
    ActionServer(ros.node, Fibonacci, "fibonacci/compute", default_execute_callback)
    compute_fibonacci: FibonacciActionable = Actionable(Fibonacci, "fibonacci/compute", ros.node)
    assert compute_fibonacci.wait_for_server(timeout_sec=2.0)
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
    assert action.status == GoalStatus.STATUS_SUCCEEDED
    expected_result = array.array("i", [0, 1, 1, 2, 3, 5])
    assert action.result.sequence == expected_result
    stored_sequences = [feedback.sequence for feedback in action.feedback]
    assert stored_sequences == streamed_sequences


def test_successful_asynchronous_action_invocation_with_forward_feedback(ros: ROSAwareScope) -> None:
    assert ros.executor is not None
    semaphore = threading.Semaphore(0)

    def execute_callback(goal_handle: ServerGoalHandle) -> Fibonacci.Result:
        feedback = Fibonacci.Feedback()

        nonlocal semaphore
        for number in fibonacci_sequence(goal_handle.request.order):
            semaphore.acquire()
            feedback.sequence.append(number)
            goal_handle.publish_feedback(feedback)

        semaphore.acquire()
        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = feedback.sequence
        return result

    ActionServer(ros.node, Fibonacci, "fibonacci/compute", execute_callback)
    compute_fibonacci: FibonacciActionable = Actionable(Fibonacci, "fibonacci/compute", ros.node)
    assert compute_fibonacci.wait_for_server(timeout_sec=2.0)
    action = compute_fibonacci.asynchronously(Fibonacci.Goal(order=1), track_feedback=True)
    assert wait_for_future(action.acknowledgement, timeout_sec=10.0)
    first_feedback_stream = action.feedback_stream(forward_only=True, timeout_sec=5.0)
    semaphore.release()  # let some feedback out
    first_feedback = next(first_feedback_stream)
    second_feedback_stream = action.feedback_stream(forward_only=True, timeout_sec=5.0)
    semaphore.release()  # let the rest of the feedback out
    second_feedback = next(second_feedback_stream)
    semaphore.release()  # let the action complete
    assert wait_for_future(action.finalization, timeout_sec=10.0)
    assert action.finalized
    assert action.succeeded
    assert not action.aborted
    assert not action.cancelled
    assert action.status == GoalStatus.STATUS_SUCCEEDED
    assert len(first_feedback.sequence) < len(second_feedback.sequence)


def test_successful_asynchronous_action_invocation_with_limited_feedback(ros: ROSAwareScope) -> None:
    ActionServer(ros.node, Fibonacci, "fibonacci/compute", default_execute_callback)
    compute_fibonacci: FibonacciActionable = Actionable(Fibonacci, "fibonacci/compute", ros.node)
    assert compute_fibonacci.wait_for_server(timeout_sec=2.0)
    action = compute_fibonacci.asynchronously(Fibonacci.Goal(order=5), track_feedback=1)
    assert wait_for_future(action.acknowledgement, timeout_sec=10.0)
    *_, last_feedback = action.feedback_stream(buffer_size=10, timeout_sec=2.0)
    assert action.finalized
    assert action.succeeded
    assert not action.aborted
    assert not action.cancelled
    assert action.status == GoalStatus.STATUS_SUCCEEDED
    expected_result = array.array("i", [0, 1, 1, 2, 3, 5])
    assert action.result.sequence == expected_result
    assert len(action.feedback) == 1
    assert action.feedback[-1].sequence == last_feedback.sequence


def test_successful_asynchronous_action_invocation_with_ephemeral_feedback(ros: ROSAwareScope) -> None:
    semaphore = threading.Semaphore(0)

    def synchronized_execute_callback(goal_handle: ServerGoalHandle) -> Fibonacci.Result:
        nonlocal semaphore
        semaphore.acquire()
        return default_execute_callback(goal_handle)

    ActionServer(ros.node, Fibonacci, "fibonacci/compute", synchronized_execute_callback)
    compute_fibonacci: FibonacciActionable = Actionable(Fibonacci, "fibonacci/compute", ros.node)
    assert compute_fibonacci.wait_for_server(timeout_sec=2.0)
    action = compute_fibonacci.asynchronously(Fibonacci.Goal(order=5), track_feedback=0)
    assert wait_for_future(action.acknowledgement, timeout_sec=10.0)
    semaphore.release()
    *_, last_feedback = action.feedback_stream(buffer_size=10, timeout_sec=2.0)
    assert action.finalized
    assert action.succeeded
    assert not action.aborted
    assert not action.cancelled
    assert action.status == GoalStatus.STATUS_SUCCEEDED
    expected_result = array.array("i", [0, 1, 1, 2, 3, 5])
    assert action.result.sequence == expected_result
    assert len(last_feedback.sequence) > 0
    assert len(action.feedback) == 0


def test_rejected_synchronous_action_invocation(ros: ROSAwareScope) -> None:
    ActionServer(
        ros.node,
        Fibonacci,
        "fibonacci/compute",
        default_execute_callback,
        goal_callback=lambda _: GoalResponse.REJECT,
    )
    compute_fibonacci: FibonacciActionable = Actionable(Fibonacci, "fibonacci/compute", ros.node)
    assert compute_fibonacci.wait_for_server(timeout_sec=2.0)
    goal = Fibonacci.Goal(order=5)
    assert compute_fibonacci(goal, nothrow=True, timeout_sec=10.0) is None
    with pytest.raises(ActionRejected) as exc_info:
        compute_fibonacci(goal, timeout_sec=10.0)
    assert not exc_info.value.action.accepted


def test_rejected_asynchronous_action_invocation(ros: ROSAwareScope) -> None:
    ActionServer(
        ros.node,
        Fibonacci,
        "fibonacci/compute",
        default_execute_callback,
        goal_callback=lambda _: GoalResponse.REJECT,
    )
    compute_fibonacci: FibonacciActionable = Actionable(Fibonacci, "fibonacci/compute", ros.node)
    assert compute_fibonacci.wait_for_server(timeout_sec=2.0)
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
        return Fibonacci.Result()

    ActionServer(ros.node, Fibonacci, "fibonacci/compute", execute_callback)
    compute_fibonacci: FibonacciActionable = Actionable(Fibonacci, "fibonacci/compute", ros.node)
    assert compute_fibonacci.wait_for_server(timeout_sec=2.0)
    goal = Fibonacci.Goal(order=5)
    result = compute_fibonacci(goal, nothrow=True, timeout_sec=10.0)
    assert len(result.sequence) == 0

    with pytest.raises(ActionAborted) as exc_info:
        compute_fibonacci(goal, timeout_sec=10.0)
    assert exc_info.value.action.aborted
    result = exc_info.value.action.result
    assert len(result.sequence) == 0


def test_aborted_asynchronous_action_invocation(ros: ROSAwareScope) -> None:
    def execute_callback(goal_handle: ServerGoalHandle) -> Fibonacci.Result:
        goal_handle.abort()
        return Fibonacci.Result()

    ActionServer(ros.node, Fibonacci, "fibonacci/compute", execute_callback)
    compute_fibonacci: FibonacciActionable = Actionable(Fibonacci, "fibonacci/compute", ros.node)
    assert compute_fibonacci.wait_for_server(timeout_sec=2.0)
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
        ros.node,
        Fibonacci,
        "fibonacci/compute",
        execute_callback,
        cancel_callback=lambda _: CancelResponse.ACCEPT,
    )
    compute_fibonacci: FibonacciActionable = Actionable(Fibonacci, "fibonacci/compute", ros.node)
    assert compute_fibonacci.wait_for_server(timeout_sec=2.0)

    assert ros.node is not None
    cancel_goal_client = ros.node.create_client(CancelGoal, "fibonacci/compute/_action/cancel_goal")

    def deferred_cancel() -> None:
        time.sleep(0.5)
        cancel_goal_client.call(CancelGoal.Request())

    goal = Fibonacci.Goal(order=5)
    assert ros.executor is not None
    ros.executor.create_task(deferred_cancel)
    result = compute_fibonacci(goal, nothrow=True, timeout_sec=10.0)
    assert len(result.sequence) == 0

    ros.executor.create_task(deferred_cancel)
    with pytest.raises(ActionCancelled) as exc_info:
        compute_fibonacci(goal, timeout_sec=10.0)
    assert exc_info.value.action.cancelled
    result = exc_info.value.action.result
    assert len(result.sequence) == 0


def test_cancelled_asynchronous_action_invocation(ros: ROSAwareScope) -> None:
    def execute_callback(goal_handle: ServerGoalHandle) -> Fibonacci.Result:
        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Fibonacci.Result()
            time.sleep(0.01)

    ActionServer(
        ros.node,
        Fibonacci,
        "fibonacci/compute",
        execute_callback,
        cancel_callback=lambda _: CancelResponse.ACCEPT,
    )
    compute_fibonacci: FibonacciActionable = Actionable(Fibonacci, "fibonacci/compute", ros.node)
    assert compute_fibonacci.wait_for_server(timeout_sec=2.0)
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
