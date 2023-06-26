import multiprocessing
import time
import unittest
from threading import Thread
from typing import Optional
import array
import rclpy
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from rclpy import Context
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from rclpy.action import ActionServer
from rclpy.action.server import GoalResponse, default_goal_callback
from bdai_ros2_wrappers.action_client import ActionClientWrapper
from action_tutorials_interfaces.action import Fibonacci


def do_not_accept_goal(goal_request):
    """
    Helper callback function for rejecting goals to help test
    """
    return GoalResponse.REJECT


class FibonacciActionServer(Node):
    """
    Action server to used for testing mostly pulled from ROS2 Action Server tutorial

    Some changes made to allow special testing of timeouts and goal rejections
    """
    def __init__(self, name: str, context: Context, test_reject=False):
        super().__init__('fibonacci_action_server', context=context)
        goal_callback = default_goal_callback if not test_reject else do_not_accept_goal
        self._action_server = ActionServer(
            self,
            Fibonacci,
            name,
            self.execute_callback,
            goal_callback=goal_callback)
        self._executor = MultiThreadedExecutor(num_threads=2, context=context)
        self._executor.add_node(self)
        self._thread = Thread(target=self._spin)
        self._thread.start()

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        # sleep to test timeout
        time.sleep(2)

        sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            sequence.append(sequence[i] + sequence[i - 1])

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = sequence
        return result

    def _spin(self):
        if self._executor is not None:
            try:
                self._executor.spin()
            except (ExternalShutdownException, KeyboardInterrupt):
                pass

    def stop(self) -> None:
        if self._executor is not None:
            self._executor.shutdown()
            self._executor.remove_node(self)
            self._executor = None
        if self._thread is not None:
            self._thread.join()
            self._thread = None
        self._action_server = None


class ActionClientWrapperTest(unittest.TestCase):
    def setUp(self) -> None:
        self.context: Optional[Context] = Context()
        rclpy.init(context=self.context)

        self.fib_action_server = FibonacciActionServer(name='fibonacci', context=self.context)
        self.fib_reject_server = FibonacciActionServer(name='reject_fib', context=self.context, test_reject=True)
        self.action_client_wrapper = ActionClientWrapper(Fibonacci, 'fibonacci', "test_action_client",
                                                         context=self.context)
        self.action_client_rejected_wrapper = ActionClientWrapper(Fibonacci, 'reject_fib', "test_rejected", context=self.context)

    def tearDown(self) -> None:
        if self.fib_action_server is not None:
            self.fib_action_server.stop()
            self.fib_action_server = None

        if self.fib_reject_server is not None:
            self.fib_reject_server.stop()
            self.fib_reject_server = None

        if self.action_client_wrapper is not None:
            self.action_client_wrapper = None

        if self.action_client_rejected_wrapper is not None:
            self.action_client_rejected_wrapper = None

        rclpy.shutdown(context=self.context)
        self.context = None

    def test_send_goal_and_wait(self):
        goal = Fibonacci.Goal()
        goal.order = 5
        expected_result = array.array('i', [0, 1, 1, 2, 3, 5])
        result = self.action_client_wrapper.send_goal_and_wait(goal)
        self.assertEquals(result.result.sequence, expected_result)

    def test_timeout_send_goal_wait(self):
        goal = Fibonacci.Goal()
        goal.order = 5

        result = self.action_client_wrapper.send_goal_and_wait(goal, 1)
        # times out and since action client wrapper does not start its own thread
        # it uses rclpy spin_until_future_complete
        self.assertEquals(result, None)

    def test_goal_not_accepted(self):
        goal = Fibonacci.Goal()
        goal.order = 5

        result = self.action_client_rejected_wrapper.send_goal_and_wait(goal)
        self.assertEquals(result, None)

if __name__ == "__main__":
    unittest.main()