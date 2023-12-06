# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.
import array

from action_tutorials_interfaces.action import Fibonacci
from rclpy.action.server import ServerGoalHandle

from bdai_ros2_wrappers.action_client import ActionClientWrapper
from bdai_ros2_wrappers.process import ROSAwareScope
from bdai_ros2_wrappers.single_goal_action_server import SingleGoalActionServer


def test_single_goal_action_server(ros: ROSAwareScope) -> None:
    """
    Tests normal operation of a single action server
    """

    def execute_callback(goal_handle: ServerGoalHandle) -> Fibonacci.Result:
        """
        Executor for normal fibonacci sequence
        """
        sequence = [0, 1]
        for i in range(1, goal_handle.request.order):
            sequence.append(sequence[i] + sequence[i - 1])

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = sequence
        return result

    assert ros.node is not None
    SingleGoalActionServer(ros.node, Fibonacci, "fibonacci", execute_callback)
    action_client = ActionClientWrapper(Fibonacci, "fibonacci", ros.node)

    goal = Fibonacci.Goal()
    goal.order = 5
    result = action_client.send_goal_and_wait(
        "test_single_goal_action_server", goal=goal, timeout_sec=2
    )
    assert result is not None

    expected_result = array.array("i", [0, 1, 1, 2, 3, 5])
    assert result.sequence == expected_result
