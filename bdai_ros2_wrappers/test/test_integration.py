# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.
import math
import random
from typing import Any, Generator

import pytest
import rclpy
import tf2_ros
from action_tutorials_interfaces.action import Fibonacci
from example_interfaces.srv import AddTwoInts
from geometry_msgs.msg import TransformStamped
from rclpy.action.server import ServerGoalHandle
from rclpy.context import Context
from rclpy.duration import Duration
from rclpy.executors import Executor
from rclpy.node import Node
from rclpy.time import Time

from bdai_ros2_wrappers.action_client import FriendlyActionClient
from bdai_ros2_wrappers.action_server import FriendlyActionServer
from bdai_ros2_wrappers.executors import AutoScalingMultiThreadedExecutor
from bdai_ros2_wrappers.node import FriendlyNode


class MinimalTransformPublisher(FriendlyNode):
    frame_id = "world"
    child_frame_id = "robot"

    def __init__(self, node_name: str = "minimal_transform_publisher", **kwargs: Any) -> None:
        super().__init__(node_name, **kwargs)
        self._broadcaster = tf2_ros.TransformBroadcaster(self)
        self._transform = TransformStamped()
        self._transform.header.frame_id = self.frame_id
        self._transform.child_frame_id = self.child_frame_id
        self._transform.transform.translation.x = 1.0
        self._transform.transform.translation.y = 2.0
        self._transform.transform.translation.z = 3.0
        self._transform.transform.rotation.z = math.sin(math.radians(45) / 2)
        self._transform.transform.rotation.w = math.cos(math.radians(45) / 2)

        self.callback()  # do not wait for first publish
        self._timer = self.create_timer(1, self.callback)

    def callback(self) -> None:
        self._transform.header.stamp = self.get_clock().now().to_msg()
        self._broadcaster.sendTransform(self._transform)


class MinimalServer(FriendlyNode):
    def __init__(self, node_name: str = "minimal_server", **kwargs: Any) -> None:
        super().__init__(node_name, **kwargs)
        self._service_server = self.create_service(AddTwoInts, "add_two_ints", self.service_callback)

    def service_callback(self, request: AddTwoInts.Request, response: AddTwoInts.Response) -> AddTwoInts.Response:
        response.sum = request.a + request.b
        return response


class MinimalActionServer(FriendlyNode):
    def __init__(self, node_name: str = "minimal_action_server", **kwargs: Any) -> None:
        super().__init__(node_name, **kwargs)
        self._action_server = FriendlyActionServer(self, Fibonacci, "compute_fibonacci_sequence", self.execute_callback)

    def execute_callback(self, goal_handle: ServerGoalHandle) -> Fibonacci.Result:
        sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            sequence.append(sequence[i] + sequence[i - 1])
            feedback = Fibonacci.Feedback()
            feedback.partial_sequence = sequence
            goal_handle.publish_feedback(feedback)
        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = sequence
        return result


@pytest.fixture
def ros_context() -> Generator[Context, None, None]:
    context = Context()
    rclpy.init(context=context)
    try:
        yield context
    finally:
        context.try_shutdown()


@pytest.fixture
def ros_executor(ros_context: Context) -> Generator[Executor, None, None]:
    executor = AutoScalingMultiThreadedExecutor(context=ros_context)
    try:
        yield executor
    finally:
        executor.shutdown()


@pytest.fixture
def ros_node(ros_context: Context, ros_executor: Executor) -> Generator[Node, None, None]:
    node = FriendlyNode("test_node", context=ros_context)
    try:
        yield node
    finally:
        node.destroy_node()


@pytest.fixture(autouse=True)
def ros_graph(ros_context: Context, ros_executor: Executor, ros_node: Node) -> Generator[None, None, None]:
    ros_executor.add_node(MinimalTransformPublisher(context=ros_context))
    ros_executor.add_node(MinimalServer(context=ros_context))
    ros_executor.add_node(MinimalActionServer(context=ros_context))
    ros_executor.add_node(ros_node)
    try:
        yield
    finally:
        for node in ros_executor.get_nodes():
            ros_executor.remove_node(node)


def test_blocking_sequence(ros_executor: Executor, ros_node: Node) -> None:
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer, node=ros_node, spin_thread=False)

    client = ros_node.create_client(AddTwoInts, "add_two_ints")
    action_client = FriendlyActionClient(ros_node, Fibonacci, "compute_fibonacci_sequence")

    def blocking_sequence() -> TransformStamped:
        generator = random.Random(0)
        seed = generator.randint(5, 10)
        assert client.wait_for_service(timeout_sec=5)
        response = client.call(AddTwoInts.Request(a=seed, b=3))
        assert response.sum == seed + 3

        assert action_client.wait_for_server(timeout_sec=5)
        feedback = []
        result = action_client.send_goal(
            Fibonacci.Goal(order=response.sum), feedback_callback=lambda f: feedback.append(f.feedback)
        ).result
        assert len(feedback) > 0
        partial_sequence = feedback[-1].partial_sequence
        assert result.sequence[: len(partial_sequence)] == partial_sequence

        timeout = Duration(seconds=5)
        assert tf_buffer.can_transform(
            MinimalTransformPublisher.frame_id, MinimalTransformPublisher.child_frame_id, Time(), timeout
        )

        time = ros_node.get_clock().now()
        time += Duration(seconds=result.sequence[-1] * 10e-3)
        return tf_buffer.lookup_transform(
            MinimalTransformPublisher.frame_id, MinimalTransformPublisher.child_frame_id, time, timeout
        )

    future = ros_executor.create_task(blocking_sequence)
    ros_executor.spin_until_future_complete(future, timeout_sec=10)
    transform = future.result()
    assert transform is not None
    assert transform.header.frame_id == MinimalTransformPublisher.frame_id
