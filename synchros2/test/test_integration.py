# Copyright (c) 2023 Boston Dynamics AI Institute LLC.  All rights reserved.
import math
import random
from typing import Any

import pytest
import tf2_ros
from example_interfaces.action import Fibonacci
from example_interfaces.srv import AddTwoInts
from geometry_msgs.msg import TransformStamped
from rclpy.action.client import ActionClient
from rclpy.action.server import ActionServer, ServerGoalHandle
from rclpy.duration import Duration
from rclpy.time import Time

from synchros2.futures import wait_for_future
from synchros2.node import Node
from synchros2.scope import ROSAwareScope


class MinimalTransformPublisher(Node):
    """A minimal ROS 2 node broadcasting a fixed transform over /tf."""

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


class MinimalServer(Node):
    """A minimal ROS 2 node serving an example_interfaces.srv.AddTwoInts service."""

    def __init__(self, node_name: str = "minimal_server", **kwargs: Any) -> None:
        super().__init__(node_name, **kwargs)
        self._service_server = self.create_service(AddTwoInts, "add_two_ints", self.service_callback)

    def service_callback(self, request: AddTwoInts.Request, response: AddTwoInts.Response) -> AddTwoInts.Response:
        response.sum = request.a + request.b
        return response


class MinimalActionServer(Node):
    """A minimal ROS 2 node serving an example_interfaces.action.Fibonacci action."""

    def __init__(self, node_name: str = "minimal_action_server", **kwargs: Any) -> None:
        super().__init__(node_name, **kwargs)
        self._action_server = ActionServer(self, Fibonacci, "compute_fibonacci_sequence", self.execute_callback)

    def execute_callback(self, goal_handle: ServerGoalHandle) -> Fibonacci.Result:
        sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            sequence.append(sequence[i] + sequence[i - 1])
            feedback = Fibonacci.Feedback()
            feedback.sequence = sequence
            goal_handle.publish_feedback(feedback)
        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = sequence
        return result


@pytest.fixture(autouse=True)
def ros_graph(ros: ROSAwareScope) -> None:
    """An automatic fixture managing execution of multiple nodes for testing purposes."""
    ros.load(MinimalTransformPublisher)
    ros.load(MinimalServer)
    ros.load(MinimalActionServer)
    return


def test_blocking_sequence(ros: ROSAwareScope) -> None:
    """Asserts that a blocking call sequence (single-nested if you follow the execution path
    across callbacks) is possible when using a multi-threaded executor and callback isolation.
    """
    assert ros.node is not None
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer, node=ros.node, spin_thread=False)

    client = ros.node.create_client(AddTwoInts, "add_two_ints")
    action_client = ActionClient(ros.node, Fibonacci, "compute_fibonacci_sequence")

    def blocking_sequence() -> TransformStamped:
        generator = random.Random(0)
        seed = generator.randint(5, 10)
        assert client.wait_for_service(timeout_sec=5)
        response = client.call(AddTwoInts.Request(a=seed, b=3))
        assert response.sum == seed + 3

        assert action_client.wait_for_server(timeout_sec=5)
        feedback = []
        result = action_client.send_goal(
            Fibonacci.Goal(order=response.sum),
            feedback_callback=lambda f: feedback.append(f.feedback),
        ).result
        assert len(feedback) > 0
        partial_sequence = feedback[-1].sequence
        assert result.sequence[: len(partial_sequence)] == partial_sequence

        timeout = Duration(seconds=5)
        assert tf_buffer.can_transform(
            MinimalTransformPublisher.frame_id,
            MinimalTransformPublisher.child_frame_id,
            Time(),
            timeout,
        )

        assert ros.node is not None
        time = ros.node.get_clock().now()
        time += Duration(seconds=result.sequence[-1] * 10e-3)
        return tf_buffer.lookup_transform(
            MinimalTransformPublisher.frame_id,
            MinimalTransformPublisher.child_frame_id,
            time,
            timeout,
        )

    assert ros.executor is not None
    future = ros.executor.create_task(blocking_sequence)
    assert wait_for_future(future, timeout_sec=10)
    transform = future.result()
    assert transform is not None
    assert transform.header.frame_id == MinimalTransformPublisher.frame_id


def test_chain_sequence(ros: ROSAwareScope) -> None:
    """Asserts that a chained call sequence (double-nested if you follow the execution path
    across callbacks) is possible when using a multi-threaded executor and callback isolation.
    """
    assert ros.node is not None
    action_client = ActionClient(ros.node, Fibonacci, "compute_fibonacci_sequence")

    def add_fibonacci_sequences_server_callback(
        request: AddTwoInts.Request,
        response: AddTwoInts.Response,
    ) -> AddTwoInts.Response:
        if not action_client.wait_for_server(timeout_sec=5):
            response.sum = -1
            return response
        result_a = action_client.send_goal(Fibonacci.Goal(order=request.a)).result
        result_b = action_client.send_goal(Fibonacci.Goal(order=request.b)).result
        response.sum = sum(result_a.sequence) + sum(result_b.sequence)
        return response

    ros.node.create_service(AddTwoInts, "add_two_fibonacci_sequences", add_fibonacci_sequences_server_callback)

    client = ros.node.create_client(AddTwoInts, "add_two_fibonacci_sequences")

    def chain_sequence() -> TransformStamped:
        assert client.wait_for_service(timeout_sec=5)
        response = client.call(AddTwoInts.Request(a=6, b=12))
        return response.sum == 396

    assert ros.executor is not None
    future = ros.executor.create_task(chain_sequence)
    assert wait_for_future(future, timeout_sec=10)
    assert future.result()
