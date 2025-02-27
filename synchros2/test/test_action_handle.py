# Copyright (c) 2023 Boston Dynamics AI Institute LLC.  All rights reserved.
import inspect
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Optional, Tuple
from unittest.mock import Mock

from example_interfaces.action import Fibonacci
from rclpy.action import ActionClient, ActionServer
from rclpy.action.server import CancelResponse, GoalResponse, ServerGoalHandle
from rclpy.node import Node

from synchros2.action_handle import ActionHandle
from synchros2.scope import ROSAwareScope


def _default_execute_callback(goal_handle: ServerGoalHandle) -> Fibonacci.Result:
    sequence = [0, 1]

    for i in range(1, goal_handle.request.order):
        sequence.append(sequence[i] + sequence[i - 1])

    goal_handle.succeed()

    result = Fibonacci.Result()
    result.sequence = sequence
    return result


def _execute_callback_abort(goal_handle: ServerGoalHandle) -> Fibonacci.Result:
    goal_handle.abort()
    return Fibonacci.Result()


def _execute_callback_feedback(goal_handle: ServerGoalHandle) -> Fibonacci.Result:
    feedback = Fibonacci.Feedback()
    feedback.sequence = [0, 1]
    goal_handle.publish_feedback(feedback)
    time.sleep(0.001)

    for i in range(1, goal_handle.request.order):
        feedback.sequence.append(feedback.sequence[i] + feedback.sequence[i - 1])
        goal_handle.publish_feedback(feedback)
        time.sleep(0.01)

    goal_handle.succeed()

    result = Fibonacci.Result()
    result.sequence = feedback.sequence
    return result


def _execute_callback_until_canceled(goal_handle: ServerGoalHandle) -> Fibonacci.Result:
    while True:
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return Fibonacci.Result()
        time.sleep(0.001)


def _execute_callback_slowly(goal_handle: ServerGoalHandle) -> Fibonacci.Result:
    time.sleep(0.5)
    return _default_execute_callback(goal_handle)


def _goal_callback_reject(goal: Any) -> GoalResponse:
    return GoalResponse.REJECT


def _cancel_callback_accepted(cancel_request: Any) -> CancelResponse:
    return CancelResponse.ACCEPT


class FibonacciActionServer(ActionServer):
    """Action server to used for testing mostly pulled from ROS2 Action Server tutorial

    Some changes made to allow special testing of timeouts and goal rejections
    """

    def __init__(
        self,
        node: Node,
        name: str,
        execute_callback: Callable = _default_execute_callback,
        **kwargs: Any,
    ) -> None:
        super().__init__(node, Fibonacci, name, execute_callback, **kwargs)

    def execute_callback(self, goal_handle: ServerGoalHandle) -> Fibonacci.Result:
        sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            sequence.append(sequence[i] + sequence[i - 1])

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = sequence
        return result


@dataclass
class ActionHandleMocks:
    result_callback: Mock = field(default_factory=Mock)
    failure_callback: Mock = field(default_factory=Mock)
    feedback_callback: Mock = field(default_factory=Mock)
    cancel_success_callback: Mock = field(default_factory=Mock)
    cancel_failure_callback: Mock = field(default_factory=Mock)


def do_send_goal(
    action_client: ActionClient,
    goal: Any,
    label: Optional[str] = None,
) -> Tuple[ActionHandle, ActionHandleMocks]:
    if label is None:
        label = inspect.stack()[1].function
    assert action_client.wait_for_server(1)

    mocks = ActionHandleMocks()

    handle = ActionHandle(label)
    handle.set_result_callback(mocks.result_callback)
    handle.set_on_failure_callback(mocks.failure_callback)
    handle.set_feedback_callback(mocks.feedback_callback)
    handle.set_on_cancel_success_callback(mocks.cancel_success_callback)
    handle.set_on_cancel_failure_callback(mocks.cancel_failure_callback)

    send_goal_future = action_client.send_goal_async(goal, feedback_callback=handle.get_feedback_callback)
    handle.set_send_goal_future(send_goal_future)
    return handle, mocks


def test_result(ros: ROSAwareScope) -> None:
    """Tests that the result callback is correctly called when the ActionGoal completes successfully"""
    assert ros.node is not None
    FibonacciActionServer(ros.node, "fibonacci_ah")
    action_client = ActionClient(ros.node, Fibonacci, "fibonacci_ah")
    handle, mocks = do_send_goal(action_client, Fibonacci.Goal(order=5))
    assert handle.wait_for_result(timeout_sec=1.0)
    assert handle.result is not None
    assert mocks.result_callback.called
    assert not mocks.failure_callback.called
    assert mocks.feedback_callback.call_count == 0
    assert not mocks.cancel_success_callback.called
    assert not mocks.cancel_failure_callback.called


def test_reject(ros: ROSAwareScope) -> None:
    """Tests the case where the action server rejects the goal"""
    assert ros.node is not None
    FibonacciActionServer(ros.node, "fibonacci_ah", goal_callback=_goal_callback_reject)
    action_client = ActionClient(ros.node, Fibonacci, "fibonacci_ah")
    handle, mocks = do_send_goal(action_client, Fibonacci.Goal(order=5))
    assert not handle.wait_for_result(timeout_sec=1.0)
    assert handle.result is None
    assert not mocks.result_callback.called
    assert mocks.failure_callback.called
    assert mocks.feedback_callback.call_count == 0
    assert not mocks.cancel_success_callback.called
    assert not mocks.cancel_failure_callback.called


def test_abort(ros: ROSAwareScope) -> None:
    """Tests the case where the action server aborts the goal"""
    assert ros.node is not None
    FibonacciActionServer(ros.node, "fibonacci_ah", execute_callback=_execute_callback_abort)
    action_client = ActionClient(ros.node, Fibonacci, "fibonacci_ah")
    handle, mocks = do_send_goal(action_client, Fibonacci.Goal(order=5))
    assert not handle.wait_for_result(timeout_sec=1.0)
    assert handle.result is not None
    assert not mocks.result_callback.called
    assert mocks.failure_callback.called
    assert mocks.feedback_callback.call_count == 0
    assert not mocks.cancel_success_callback.called
    assert not mocks.cancel_failure_callback.called


def test_feedback(ros: ROSAwareScope) -> None:
    assert ros.node is not None
    FibonacciActionServer(ros.node, "fibonacci_ah", execute_callback=_execute_callback_feedback)
    action_client = ActionClient(ros.node, Fibonacci, "fibonacci_ah")
    handle, mocks = do_send_goal(action_client, Fibonacci.Goal(order=5))
    assert handle.wait_for_result(timeout_sec=1.0)
    assert handle.result is not None
    assert mocks.result_callback.called
    assert not mocks.failure_callback.called
    assert mocks.feedback_callback.call_count > 0
    assert not mocks.cancel_success_callback.called
    assert not mocks.cancel_failure_callback.called


def test_cancel_success(ros: ROSAwareScope) -> None:
    assert ros.node is not None
    FibonacciActionServer(
        ros.node,
        "fibonacci_ah",
        execute_callback=_execute_callback_until_canceled,
        cancel_callback=_cancel_callback_accepted,
    )
    action_client = ActionClient(ros.node, Fibonacci, "fibonacci_ah")
    handle, mocks = do_send_goal(action_client, Fibonacci.Goal(order=5))
    assert handle.wait_for_acceptance(timeout_sec=1.0)
    handle.cancel()
    assert not handle.wait_for_result(timeout_sec=5.0)
    assert handle.result is not None
    assert not mocks.result_callback.called
    assert not mocks.failure_callback.called
    assert mocks.feedback_callback.call_count == 0
    assert mocks.cancel_success_callback.called
    assert not mocks.cancel_failure_callback.called


def test_cancel_failure(ros: ROSAwareScope) -> None:
    assert ros.node is not None
    FibonacciActionServer(ros.node, "fibonacci_ah", execute_callback=_execute_callback_slowly)
    action_client = ActionClient(ros.node, Fibonacci, "fibonacci_ah")
    handle, mocks = do_send_goal(action_client, Fibonacci.Goal(order=5))
    assert handle.wait_for_acceptance(timeout_sec=1.0)
    handle.cancel()
    assert handle.wait_for_result(timeout_sec=1.0)
    assert handle.result is not None
    assert mocks.result_callback.called
    assert not mocks.failure_callback.called
    assert mocks.feedback_callback.call_count == 0
    assert not mocks.cancel_success_callback.called
    assert mocks.cancel_failure_callback.called


def test_wait_for_result_timeout(ros: ROSAwareScope) -> None:
    """Tests the wait_for_result function for when it times out"""
    assert ros.node is not None
    FibonacciActionServer(ros.node, "fibonacci_ah", execute_callback=_execute_callback_slowly)
    action_client = ActionClient(ros.node, Fibonacci, "fibonacci_ah")
    handle, mocks = do_send_goal(action_client, Fibonacci.Goal(order=5))
    assert not handle.wait_for_result(timeout_sec=0.01)


def test_wait_for_result_cancelled(ros: ROSAwareScope) -> None:
    """Test if the ActionServer cancels the request"""
    assert ros.node is not None
    FibonacciActionServer(
        ros.node,
        "fibonacci_ah",
        execute_callback=_execute_callback_until_canceled,
        cancel_callback=_cancel_callback_accepted,
    )
    action_client = ActionClient(ros.node, Fibonacci, "fibonacci_ah")
    handle, mocks = do_send_goal(action_client, Fibonacci.Goal(order=5))
    assert handle.wait_for_acceptance(timeout_sec=1.0)
    # Cancel the ActionGoalHandle during the test
    handle.cancel()
    # Check the return value and make sure the result is both executed to completion and returns negatively
    assert not handle.wait_for_result(timeout_sec=5.0)
    assert handle.result is not None
    assert not mocks.result_callback.called
    assert not mocks.failure_callback.called
    assert mocks.feedback_callback.call_count == 0
    assert mocks.cancel_success_callback.called
    assert not mocks.cancel_failure_callback.called


def test_wait_for_result_aborted(ros: ROSAwareScope) -> None:
    """Test if the ActionServer cancels the request"""
    assert ros.node is not None
    FibonacciActionServer(ros.node, "fibonacci_ah", execute_callback=_execute_callback_abort)
    action_client = ActionClient(ros.node, Fibonacci, "fibonacci_ah")
    handle, mocks = do_send_goal(action_client, Fibonacci.Goal(order=5))
    # Check the return value and make sure the result is both executed to completion and returns negatively
    assert not handle.wait_for_result(timeout_sec=10.0)
    assert handle.result is not None
    assert not mocks.result_callback.called
    assert mocks.failure_callback.called
    assert mocks.feedback_callback.call_count == 0
    assert not mocks.cancel_success_callback.called
    assert not mocks.cancel_failure_callback.called
