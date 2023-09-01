# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.
import threading
import time
import unittest
from enum import Enum
from threading import Thread
from typing import Any, Callable, Optional, Tuple

import rclpy
from action_tutorials_interfaces.action import Fibonacci
from rclpy import Context
from rclpy.action import ActionClient, ActionServer
from rclpy.action.server import (
    CancelResponse,
    GoalResponse,
    ServerGoalHandle,
    default_cancel_callback,
    default_goal_callback,
    default_handle_accepted_callback,
)
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node

from bdai_ros2_wrappers.action_handle import ActionHandle
from bdai_ros2_wrappers.type_hints import Action

GOAL_ORDER = 5


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
    feedback.partial_sequence = [0, 1]
    goal_handle.publish_feedback(feedback)
    time.sleep(0.001)

    for i in range(1, goal_handle.request.order):
        feedback.partial_sequence.append(feedback.partial_sequence[i] + feedback.partial_sequence[i - 1])
        goal_handle.publish_feedback(feedback)
        time.sleep(0.01)

    goal_handle.succeed()

    result = Fibonacci.Result()
    result.sequence = feedback.partial_sequence
    return result


def _execute_callback_until_canceled(goal_handle: ServerGoalHandle) -> Fibonacci.Result:
    while True:
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return Fibonacci.Result()
        time.sleep(0.001)


def _execute_callback_until_canceled_or_flagged(
    flag: Callable[[], bool], goal_handle: ServerGoalHandle
) -> Fibonacci.Result:
    while flag():
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return Fibonacci.Result()
        time.sleep(0.001)
    goal_handle.succeed()
    result = Fibonacci.Result()
    result.sequence = [0, 1]
    return result


def _execute_callback_for_timeout(goal_handle: ServerGoalHandle) -> Fibonacci.Result:
    time.sleep(0.1)
    return Fibonacci.Result()


def _goal_callback_reject(goal: Action.Goal) -> GoalResponse:
    return GoalResponse.REJECT


def _cancel_callback_accepted(cancel_request: Any) -> CancelResponse:
    return CancelResponse.ACCEPT


class FibonacciActionServerNode(Node):
    def __init__(
        self,
        context: Context,
        execute_callback: Optional[Callable[[ServerGoalHandle], Fibonacci.Result]] = None,
        goal_callback: Optional[Callable[[ServerGoalHandle], GoalResponse]] = None,
        handle_accepted_callback: Optional[Callable[[ServerGoalHandle], None]] = None,
        cancel_callback: Optional[Callable[[Action.Goal], CancelResponse]] = None,
    ) -> None:
        super().__init__("server", context=context)
        self._context = context
        if execute_callback is None:
            execute_callback = _default_execute_callback
        if goal_callback is None:
            goal_callback = default_goal_callback
        if handle_accepted_callback is None:
            handle_accepted_callback = default_handle_accepted_callback
        if cancel_callback is None:
            cancel_callback = default_cancel_callback
        self._action_server = ActionServer(
            self,
            Fibonacci,
            "fibonacci_ah",
            execute_callback=execute_callback,
            goal_callback=goal_callback,
            handle_accepted_callback=handle_accepted_callback,
            cancel_callback=cancel_callback,
        )
        self._executor: Optional[MultiThreadedExecutor] = MultiThreadedExecutor(2, context=context)
        self._executor.add_node(self)
        self._keep_spinning = False
        self._thread: Optional[Thread] = Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self) -> None:
        if self._executor is not None:
            self._keep_spinning = True
            try:
                while self._context.ok() and self._keep_spinning:
                    self._executor.spin_once(timeout_sec=0.1)
            except (ExternalShutdownException, KeyboardInterrupt):
                pass
            self._keep_spinning = False

    def stop(self) -> None:
        self._keep_spinning = False
        if self._executor is not None:
            self._executor.shutdown()
            self._executor.remove_node(self)
            self._executor = None
        if self._thread is not None:
            self._thread.join()
            self._thread = None
        self._action_server = None


class CancelOption(Enum):
    IGNORE = 0
    WAIT_FOR_SUCCESS = 1
    WAIT_FOR_RESULT = 2


class ActionHandleTest(unittest.TestCase):
    def setUp(self) -> None:
        self.context = Context()
        rclpy.init(context=self.context)
        self.client_node: Optional[Node] = Node("client", context=self.context)
        self.client = ActionClient(self.client_node, Fibonacci, "fibonacci_ah")
        self.server_node: Optional[FibonacciActionServerNode] = None

    def tearDown(self) -> None:
        if self.client_node is not None:
            self.client_node.destroy_node()
            self.client_node = None
        if self.server_node is not None:
            self.server_node.stop()
            self.server_node.destroy_node()
            self.server_node = None
        rclpy.shutdown(context=self.context)
        self.context = None

    def _internal(
        self, name: str, cancel: CancelOption = CancelOption.IGNORE, wait: bool = False
    ) -> Tuple[Fibonacci.Result, bool, bool, int, bool, bool]:
        self.assertTrue(self.client.wait_for_server(1))

        result_callback_run = False

        def _result_callback(_: Fibonacci.Result) -> None:
            nonlocal result_callback_run
            result_callback_run = True

        failure_callback_run = False

        def _failure_callback() -> None:
            nonlocal failure_callback_run
            failure_callback_run = True

        num_feedback_callback_run = 0

        def _feedback_callback(_: Fibonacci.Feedback) -> None:
            nonlocal num_feedback_callback_run
            num_feedback_callback_run += 1

        cancel_success_callback_run = False

        def _cancel_success_callback() -> None:
            nonlocal cancel_success_callback_run
            cancel_success_callback_run = True

        cancel_failure_callback_run = False

        def _cancel_failure_callback() -> None:
            nonlocal cancel_failure_callback_run
            cancel_failure_callback_run = True

        handle = ActionHandle(name)
        handle.set_result_callback(_result_callback)
        handle.set_on_failure_callback(_failure_callback)
        handle.set_feedback_callback(_feedback_callback)
        handle.set_on_cancel_success_callback(_cancel_success_callback)
        handle.set_on_cancel_failure_callback(_cancel_failure_callback)

        goal = Fibonacci.Goal()
        goal.order = GOAL_ORDER

        send_goal_future = self.client.send_goal_async(goal, feedback_callback=handle.get_feedback_callback)
        handle.set_send_goal_future(send_goal_future)

        executor = SingleThreadedExecutor(context=self.context)
        executor.add_node(self.client_node)

        if cancel is not CancelOption.IGNORE:
            executor.spin_once()
            executor.spin_once()
            executor.spin_once()
            handle.cancel()
        while True:
            executor.spin_once()
            if cancel == CancelOption.IGNORE:
                if result_callback_run or failure_callback_run:
                    break
            elif cancel == CancelOption.WAIT_FOR_SUCCESS:
                if cancel_success_callback_run:
                    break
            elif cancel == CancelOption.WAIT_FOR_RESULT:
                if (cancel_success_callback_run or cancel_failure_callback_run) and result_callback_run:
                    break
        executor.remove_node(self.client_node)
        executor.shutdown()

        return (
            handle.result,
            result_callback_run,
            failure_callback_run,
            num_feedback_callback_run,
            cancel_success_callback_run,
            cancel_failure_callback_run,
        )

    def test_result(self) -> None:
        """Tests that the result callback is correctly called when the ActionGoal completes successfully"""
        self.server_node = FibonacciActionServerNode(self.context)
        (
            result,
            result_callback_run,
            failure_callback_run,
            num_feedback_callback_run,
            cancel_success_callback_run,
            cancel_failure_callback_run,
        ) = self._internal("test_result")
        self.assertTrue(result is not None)
        self.assertTrue(result_callback_run)
        self.assertFalse(failure_callback_run)
        self.assertEqual(num_feedback_callback_run, 0)
        self.assertFalse(cancel_success_callback_run)
        self.assertFalse(cancel_failure_callback_run)

    def test_reject(self) -> None:
        """Tests the case where the action server rejects the goal"""
        self.server_node = FibonacciActionServerNode(self.context, goal_callback=_goal_callback_reject)
        (
            result,
            result_callback_run,
            failure_callback_run,
            num_feedback_callback_run,
            cancel_success_callback_run,
            cancel_failure_callback_run,
        ) = self._internal("test_reject")
        self.assertTrue(result is None)
        self.assertFalse(result_callback_run)
        self.assertTrue(failure_callback_run)
        self.assertEqual(num_feedback_callback_run, 0)
        self.assertFalse(cancel_success_callback_run)
        self.assertFalse(cancel_failure_callback_run)

    def test_abort(self) -> None:
        """Tests the case where the action server aborts the goal"""
        self.server_node = FibonacciActionServerNode(self.context, execute_callback=_execute_callback_abort)
        (
            result,
            result_callback_run,
            failure_callback_run,
            num_feedback_callback_run,
            cancel_success_callback_run,
            cancel_failure_callback_run,
        ) = self._internal("test_abort")
        self.assertTrue(result is not None)
        self.assertFalse(result_callback_run)
        self.assertTrue(failure_callback_run)
        self.assertEqual(num_feedback_callback_run, 0)
        self.assertFalse(cancel_success_callback_run)
        self.assertFalse(cancel_failure_callback_run)

    def test_feedback(self) -> None:
        self.server_node = FibonacciActionServerNode(self.context, execute_callback=_execute_callback_feedback)
        (
            result,
            result_callback_run,
            failure_callback_run,
            num_feedback_callback_run,
            cancel_success_callback_run,
            cancel_failure_callback_run,
        ) = self._internal("test_feedback")
        self.assertTrue(result is not None)
        self.assertTrue(result_callback_run)
        self.assertFalse(failure_callback_run)
        self.assertEqual(num_feedback_callback_run, GOAL_ORDER)
        self.assertFalse(cancel_success_callback_run)
        self.assertFalse(cancel_failure_callback_run)

    def test_cancel_success(self) -> None:
        self.server_node = FibonacciActionServerNode(
            self.context, execute_callback=_execute_callback_until_canceled, cancel_callback=_cancel_callback_accepted
        )
        (
            result,
            result_callback_run,
            failure_callback_run,
            num_feedback_callback_run,
            cancel_success_callback_run,
            cancel_failure_callback_run,
        ) = self._internal("test_cancel_success", cancel=CancelOption.WAIT_FOR_SUCCESS)
        self.assertTrue(result is not None)
        self.assertFalse(result_callback_run)
        self.assertFalse(failure_callback_run)
        self.assertEqual(num_feedback_callback_run, 0)
        self.assertTrue(cancel_success_callback_run)
        self.assertFalse(cancel_failure_callback_run)

    def test_cancel_failure(self) -> None:
        self.server_node = FibonacciActionServerNode(self.context, execute_callback=_default_execute_callback)
        (
            result,
            result_callback_run,
            failure_callback_run,
            num_feedback_callback_run,
            cancel_success_callback_run,
            cancel_failure_callback_run,
        ) = self._internal("test_cancel_failure", cancel=CancelOption.WAIT_FOR_RESULT)
        self.assertTrue(result is not None)
        self.assertTrue(result_callback_run)
        self.assertFalse(failure_callback_run)
        self.assertEqual(num_feedback_callback_run, 0)
        self.assertFalse(cancel_success_callback_run)
        self.assertTrue(cancel_failure_callback_run)

    def test_wait_for_result(self) -> None:
        """Tests the wait_for_result function"""
        self.server_node = FibonacciActionServerNode(self.context)

        result_callback_run = False

        def _result_callback(_: Fibonacci.Result) -> None:
            nonlocal result_callback_run
            result_callback_run = True

        failure = False

        def _failure_callback() -> None:
            nonlocal failure
            failure = True

        num_feedback_callback_run = 0

        def _feedback_callback(_: Fibonacci.Feedback) -> None:
            nonlocal num_feedback_callback_run
            num_feedback_callback_run += 1

        cancel_success_callback_run = False

        def _cancel_success_callback() -> None:
            nonlocal cancel_success_callback_run
            cancel_success_callback_run = True

        cancel_failure_callback_run = False

        def _cancel_failure_callback() -> None:
            nonlocal cancel_failure_callback_run
            cancel_failure_callback_run = True

        handle = ActionHandle("test_wait_for_result")
        handle.set_result_callback(_result_callback)
        handle.set_on_failure_callback(_failure_callback)
        handle.set_feedback_callback(_feedback_callback)
        handle.set_on_cancel_success_callback(_cancel_success_callback)
        handle.set_on_cancel_failure_callback(_cancel_failure_callback)

        goal = Fibonacci.Goal()
        goal.order = GOAL_ORDER

        send_goal_future = self.client.send_goal_async(goal, feedback_callback=handle.get_feedback_callback)
        handle.set_send_goal_future(send_goal_future)

        executor = SingleThreadedExecutor(context=self.context)
        executor.add_node(self.client_node)

        keep_spinning = True

        def _spin_executor() -> None:
            nonlocal executor
            nonlocal keep_spinning
            try:
                while self.context.ok() and keep_spinning:
                    executor.spin_once(timeout_sec=0.1)
            except (ExternalShutdownException, KeyboardInterrupt):
                pass

        t = threading.Thread(target=_spin_executor, daemon=True)
        t.start()
        self.assertTrue(handle.wait_for_result())
        keep_spinning = False

        executor.shutdown()
        executor.remove_node(self.client_node)

        t.join()
        self.assertTrue(handle.result is not None)
        self.assertTrue(result_callback_run)
        self.assertFalse(failure)
        self.assertEqual(num_feedback_callback_run, 0)
        self.assertFalse(cancel_success_callback_run)
        self.assertFalse(cancel_failure_callback_run)

    def test_wait_for_result_timeout(self) -> None:
        """Tests the wait_for_result function for when it times out"""
        self.server_node = FibonacciActionServerNode(self.context, execute_callback=_execute_callback_for_timeout)

        result_callback_run = False

        def _result_callback(_: Fibonacci.Result) -> None:
            nonlocal result_callback_run
            result_callback_run = True

        failure_callback_run = False

        def _failure_callback() -> None:
            nonlocal failure_callback_run
            failure_callback_run = True

        num_feedback_callback_run = 0

        def _feedback_callback(_: Fibonacci.Feedback) -> None:
            nonlocal num_feedback_callback_run
            num_feedback_callback_run += 1

        cancel_success_callback_run = False

        def _cancel_success_callback() -> None:
            print("Cancel success")
            nonlocal cancel_success_callback_run
            cancel_success_callback_run = True

        cancel_failure_callback_run = False

        def _cancel_failure_callback() -> None:
            nonlocal cancel_failure_callback_run
            cancel_failure_callback_run = True

        handle = ActionHandle("test_wait_for_result")
        handle.set_result_callback(_result_callback)
        handle.set_on_failure_callback(_failure_callback)
        handle.set_feedback_callback(_feedback_callback)
        handle.set_on_cancel_success_callback(_cancel_success_callback)
        handle.set_on_cancel_failure_callback(_cancel_failure_callback)

        goal = Fibonacci.Goal()
        goal.order = GOAL_ORDER

        send_goal_future = self.client.send_goal_async(goal, feedback_callback=handle.get_feedback_callback)
        handle.set_send_goal_future(send_goal_future)

        executor = SingleThreadedExecutor(context=self.context)
        executor.add_node(self.client_node)

        keep_spinning = True

        def _spin_executor() -> None:
            nonlocal executor
            nonlocal keep_spinning
            try:
                while self.context.ok() and keep_spinning:
                    executor.spin_once(timeout_sec=0.1)
            except (ExternalShutdownException, KeyboardInterrupt):
                pass

        t = threading.Thread(target=_spin_executor, daemon=True)
        t.start()
        self.assertFalse(handle.wait_for_result(timeout_sec=0.01))
        keep_spinning = False
        executor.shutdown()
        executor.remove_node(self.client_node)
        t.join()

        self.assertTrue(handle.result is None)
        self.assertFalse(result_callback_run)
        self.assertFalse(failure_callback_run)
        self.assertEqual(num_feedback_callback_run, 0)
        self.assertFalse(cancel_success_callback_run)
        self.assertFalse(cancel_failure_callback_run)

    def test_wait_for_result_cancelled(self) -> None:
        """Test if the ActionServer cancels the request"""
        self.server_node = FibonacciActionServerNode(
            self.context, execute_callback=_execute_callback_until_canceled, cancel_callback=_cancel_callback_accepted
        )

        result_callback_run = False

        def _result_callback(_: Fibonacci.Result) -> None:
            nonlocal result_callback_run
            result_callback_run = True

        failure_callback_run = False

        def _failure_callback() -> None:
            nonlocal failure_callback_run
            failure_callback_run = True

        num_feedback_callback_run = 0

        def _feedback_callback(_: Fibonacci.Feedback) -> None:
            nonlocal num_feedback_callback_run
            num_feedback_callback_run += 1

        cancel_success_callback_run = False

        def _cancel_success_callback() -> None:
            nonlocal cancel_success_callback_run
            cancel_success_callback_run = True

        cancel_failure_callback_run = False

        def _cancel_failure_callback() -> None:
            nonlocal cancel_failure_callback_run
            cancel_failure_callback_run = True

        handle = ActionHandle("test_wait_for_result_cancelled")
        handle.set_result_callback(_result_callback)
        handle.set_on_failure_callback(_failure_callback)
        handle.set_feedback_callback(_feedback_callback)
        handle.set_on_cancel_success_callback(_cancel_success_callback)
        handle.set_on_cancel_failure_callback(_cancel_failure_callback)

        goal = Fibonacci.Goal()
        goal.order = GOAL_ORDER

        send_goal_future = self.client.send_goal_async(goal, feedback_callback=handle.get_feedback_callback)
        handle.set_send_goal_future(send_goal_future)

        executor = SingleThreadedExecutor(context=self.context)
        executor.add_node(self.client_node)

        keep_spinning = True

        def _spin_executor() -> None:
            nonlocal executor
            nonlocal keep_spinning
            try:
                while self.context.ok() and keep_spinning:
                    executor.spin_once(timeout_sec=0.1)
            except (ExternalShutdownException, KeyboardInterrupt):
                pass

        t = threading.Thread(target=_spin_executor, daemon=True)
        t.start()
        # Cancel the ActionGoalHandle during the test
        handle.cancel()
        # Check the return value and make sure the result is both executed to completion and returns negatively
        self.assertFalse(handle.wait_for_result())
        keep_spinning = False

        executor.shutdown()
        executor.remove_node(self.client_node)

        t.join()
        self.assertTrue(handle.result is not None)
        self.assertFalse(result_callback_run)
        self.assertFalse(failure_callback_run)
        self.assertEqual(num_feedback_callback_run, 0)
        self.assertTrue(cancel_success_callback_run)
        self.assertFalse(cancel_failure_callback_run)

    def test_wait_for_result_aborted(self) -> None:
        """Test if the ActionServer cancels the request"""
        self.server_node = FibonacciActionServerNode(self.context, execute_callback=_execute_callback_abort)

        result_callback_run = False

        def _result_callback(_: Fibonacci.Result) -> None:
            nonlocal result_callback_run
            result_callback_run = True

        failure_callback_run = False

        def _failure_callback() -> None:
            nonlocal failure_callback_run
            failure_callback_run = True

        num_feedback_callback_run = 0

        def _feedback_callback(_: Fibonacci.Feedback) -> None:
            nonlocal num_feedback_callback_run
            num_feedback_callback_run += 1

        cancel_success_callback_run = False

        def _cancel_success_callback() -> None:
            nonlocal cancel_success_callback_run
            cancel_success_callback_run = True

        cancel_failure_callback_run = False

        def _cancel_failure_callback() -> None:
            nonlocal cancel_failure_callback_run
            cancel_failure_callback_run = True

        handle = ActionHandle("test_wait_for_result_aborted")
        handle.set_result_callback(_result_callback)
        handle.set_on_failure_callback(_failure_callback)
        handle.set_feedback_callback(_feedback_callback)
        handle.set_on_cancel_success_callback(_cancel_success_callback)
        handle.set_on_cancel_failure_callback(_cancel_failure_callback)

        goal = Fibonacci.Goal()
        goal.order = GOAL_ORDER

        send_goal_future = self.client.send_goal_async(goal, feedback_callback=handle.get_feedback_callback)
        handle.set_send_goal_future(send_goal_future)

        executor = SingleThreadedExecutor(context=self.context)
        executor.add_node(self.client_node)

        keep_spinning = True

        def _spin_executor() -> None:
            nonlocal executor
            nonlocal keep_spinning
            try:
                while self.context.ok() and keep_spinning:
                    executor.spin_once(timeout_sec=0.1)
            except (ExternalShutdownException, KeyboardInterrupt):
                pass

        t = threading.Thread(target=_spin_executor, daemon=True)
        t.start()
        # Cancel the ActionGoalHandle during the test
        handle.cancel()
        # Check the return value and make sure the result is both executed to completion and returns negatively
        self.assertFalse(handle.wait_for_result())
        keep_spinning = False

        executor.shutdown()
        executor.remove_node(self.client_node)

        t.join()
        self.assertTrue(handle.result is not None)
        self.assertFalse(result_callback_run)
        self.assertTrue(failure_callback_run)
        self.assertEqual(num_feedback_callback_run, 0)
        self.assertFalse(cancel_success_callback_run)
        self.assertFalse(cancel_failure_callback_run)


if __name__ == "__main__":
    unittest.main()
