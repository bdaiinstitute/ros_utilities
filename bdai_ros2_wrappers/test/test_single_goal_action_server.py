# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.
import array
import unittest
from typing import Optional

import rclpy
from action_tutorials_interfaces.action import Fibonacci
from rclpy import Context
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import CallbackGroup, ReentrantCallbackGroup

from bdai_ros2_wrappers.action_client import ActionClientWrapper
from bdai_ros2_wrappers.node import NodeWrapper
from bdai_ros2_wrappers.single_goal_action_server import SingleGoalActionServer


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


class SingleGoalActionServerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.context: Optional[Context] = Context()
        rclpy.init(context=self.context)
        self.group: CallbackGroup = ReentrantCallbackGroup()
        self.action_server_node: NodeWrapper = NodeWrapper(
            "server", context=self.context, num_executor_threads=2, spin_thread=True
        )
        self.action_server: SingleGoalActionServer = SingleGoalActionServer(
            self.action_server_node,
            Fibonacci,
            "fibonacci",
            execute_callback=execute_callback,
            callback_group=self.group,
        )
        self.client: ActionClientWrapper = ActionClientWrapper(
            Fibonacci, "fibonacci", "test_action_client", context=self.context
        )

    def tearDown(self) -> None:
        if self.client is not None:
            self.client = None
        if self.action_server is not None:
            self.action_server.destroy()
            self.action_server = None
        if self.action_server_node is not None:
            self.action_server_node.shutdown()

        rclpy.shutdown(context=self.context)
        self.context = None

    def test_single_goal_action_server(self) -> None:
        """
        Tests normal operation of a single action server
        """
        goal = Fibonacci.Goal()
        goal.order = 5
        expected_result = array.array("i", [0, 1, 1, 2, 3, 5])
        result = self.client.send_goal_and_wait("test_single_goal_action_server", goal=goal, timeout_sec=2)
        self.assertEqual(result.sequence, expected_result)


if __name__ == "__main__":
    unittest.main()
