import array
import time
import unittest
from threading import Thread
from typing import Optional

import rclpy
from action_tutorials_interfaces.action import Fibonacci
from rclpy import Context
from rclpy.action.server import GoalStatus, ServerGoalHandle
from rclpy.callback_groups import CallbackGroup, ReentrantCallbackGroup

from bdai_ros2_wrappers.action_client import ActionClientWrapper
from bdai_ros2_wrappers.node import NodeWrapper
from bdai_ros2_wrappers.single_goal_multiple_action_servers import SingleGoalMultipleActionServers


def execute_callback(goal_handle: ServerGoalHandle) -> Fibonacci.Result:
    """
    Executor callback for a server that does
    fibonacci
    """
    sequence = [0, 1]
    for i in range(1, goal_handle.request.order):
        sequence.append(sequence[i] + sequence[i - 1])

    goal_handle.succeed()

    result = Fibonacci.Result()
    result.sequence = sequence
    return result


def execute_callback_wrong_fib(goal_handle: ServerGoalHandle) -> Fibonacci.Result:
    """
    Different executor for another server that does
    fibonacci wrong
    """
    # time delay to make interrupting easier
    time.sleep(1)
    sequence = [0, 1]
    for i in range(1, goal_handle.request.order):
        sequence.append(sequence[i] * sequence[i - 1])

    result = None
    if goal_handle.status != GoalStatus.STATUS_ABORTED:
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = sequence
    else:
        result = Fibonacci.Result()
        result.sequence = [-1]

    return result


def threaded_client_request(client: ActionClientWrapper) -> None:
    """
    Threaded client request process to allow testing of multithreading tests
    """
    # time delay to give other action time to get started before interrupting
    time.sleep(0.3)
    goal = Fibonacci.Goal()
    goal.order = 5
    client.send_goal_and_wait(goal, 2)


class SingleGoalMultipleActionServerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.context: Optional[Context] = Context()
        rclpy.init(context=self.context)
        self.group: CallbackGroup = ReentrantCallbackGroup()
        self.action_server_node: Optional[NodeWrapper] = NodeWrapper(
            "server", context=self.context, num_executor_threads=2, spin_thread=True
        )
        self.action_params = [
            (Fibonacci, "fibonacci", execute_callback, self.group),
            (Fibonacci, "fibonacci_wrong", execute_callback_wrong_fib, self.group),
        ]
        self.action_server = SingleGoalMultipleActionServers(self.action_server_node, self.action_params)
        self.client = ActionClientWrapper(Fibonacci, "fibonacci", "test_action_client", context=self.context)
        self.client2 = ActionClientWrapper(Fibonacci, "fibonacci_wrong", "test_wrong_fib_client", context=self.context)

    def tearDown(self) -> None:
        if self.client is not None:
            self.client = None
        if self.client2 is not None:
            self.client2 = None
        if self.action_server is not None:
            self.action_server.destroy()
            self.action_server = None
        if self.action_server_node is not None:
            self.action_server_node.shutdown()

        rclpy.shutdown(context=self.context)
        self.context = None

    def test_single_goal_action_server(self) -> None:
        """
        Tests out normal operation with multiple action servers and clients
        """
        goal = Fibonacci.Goal()
        goal.order = 5
        expected_result = array.array("i", [0, 1, 1, 2, 3, 5])
        # use first client
        result = self.client.send_goal_and_wait(goal, timeout_sec=5)
        self.assertEquals(result.result.sequence, expected_result)
        # use second client
        expected_result = array.array("i", [0, 1, 0, 0, 0, 0])
        result = self.client2.send_goal_and_wait(goal, timeout_sec=5)
        self.assertEquals(result.result.sequence, expected_result)

    def test_interrupted_action(self) -> None:
        """
        This test should start a delayed request from another client
        then make an immediate request to interrupt the last request.

        Due to the threading and reliance on sleeps this test might be tempermental on other machines
        """
        goal = Fibonacci.Goal()
        goal.order = 5
        # set up thread that will interrupt this request
        thread = Thread(target=threaded_client_request, args=(self.client,))
        thread.start()
        # immediately start the request for other goal
        result = self.client2.send_goal_and_wait(goal, 5)
        # expected result when interrupted/aborted is set to be [-1] in executed callback
        expected_result = array.array("i", [-1])
        self.assertEquals(result.result.sequence, expected_result)

        # wait for thread to finish
        thread.join()


if __name__ == "__main__":
    unittest.main()
