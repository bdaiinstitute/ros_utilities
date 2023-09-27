# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.
import array
import time
from typing import Any, Iterable

import pytest
import rclpy
from action_tutorials_interfaces.action import Fibonacci
from rclpy.action.server import ActionServer, GoalResponse, ServerGoalHandle

from bdai_ros2_wrappers.action_client import ActionClientWrapper
from bdai_ros2_wrappers.node import Node
from bdai_ros2_wrappers.process import ROSAwareScope


class FibonacciActionServer(ActionServer):
    """
    Action server to used for testing mostly pulled from ROS2 Action Server tutorial

    Some changes made to allow special testing of timeouts and goal rejections
    """

    def __init__(self, node: Node, name: str, **kwargs: Any) -> None:
        super().__init__(node, Fibonacci, name, self.execute_callback, **kwargs)
        self._logger = node.get_logger()

    def execute_callback(self, goal_handle: ServerGoalHandle) -> Fibonacci.Result:
        self._logger.info("Executing goal...")
        # sleep to test timeout
        time.sleep(2)

        sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            sequence.append(sequence[i] + sequence[i - 1])

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = sequence
        return result


@pytest.fixture
def ros() -> Iterable[ROSAwareScope]:
    rclpy.init()
    try:
        with ROSAwareScope("fixture") as scope:
            yield scope
    finally:
        rclpy.try_shutdown()


def test_send_goal_and_wait(ros: ROSAwareScope) -> None:
    """
    Test standard operation of send_goal_and_wait
    """
    FibonacciActionServer(ros.node, "fibonacci")
    action_client = ActionClientWrapper(Fibonacci, "fibonacci", ros.node)

    goal = Fibonacci.Goal()
    goal.order = 5
    result = action_client.send_goal_and_wait("test_send_goal_and_wait", goal=goal, timeout_sec=5)
    assert result is not None
    expected_result = array.array("i", [0, 1, 1, 2, 3, 5])
    assert result.sequence == expected_result


def test_timeout_send_goal_wait(ros: ROSAwareScope) -> None:
    """
    Test out the timeout of the send_goal_and_wait
    """
    FibonacciActionServer(ros.node, "fibonacci")
    action_client = ActionClientWrapper(Fibonacci, "fibonacci", ros.node)

    goal = Fibonacci.Goal()
    goal.order = 5
    result = action_client.send_goal_and_wait("test_timeout_send_goal_wait", goal=goal, timeout_sec=0.5)

    # times out and since action client wrapper does not start its own thread
    # it uses rclpy spin_until_future_complete which will return none
    assert result is None


def test_goal_not_accepted(ros: ROSAwareScope) -> None:
    """
    Test for the goal not accepted pathway should return None
    """

    def do_not_accept_goal(goal_request: Fibonacci.Goal) -> GoalResponse:
        """
        Helper callback function for rejecting goals to help test
        """
        return GoalResponse.REJECT

    FibonacciActionServer(ros.node, "fibonacci", goal_callback=do_not_accept_goal)
    action_client = ActionClientWrapper(Fibonacci, "fibonacci", ros.node)

    goal = Fibonacci.Goal()
    goal.order = 5
    result = action_client.send_goal_and_wait("test_goal_not_accepted", goal=goal, timeout_sec=5)
    assert result is None
