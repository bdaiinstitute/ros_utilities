# Copyright [2023] Boston Dynamics AI Institute, Inc.
from typing import Optional

from rclpy import Context, Future
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


class NodeWrapper:
    """A wrapper around a node and its executor. The node is spun by the executor when calling
    spin_until_future_complete, which allows a future to be spun inside a callback."""

    def __init__(self, node_name: str, namespace: str = None, context: Context = None):
        self.node = Node(node_name, namespace=namespace, context=context)
        self.__executor = SingleThreadedExecutor(context=context)
        self.__executor.add_node(self.node)

    def spin_until_future_complete(self, future: Future, timeout_sec: Optional[float] = None):
        """Spin the internal node until the future completes, and return the result. This is safe to call from a
        callback."""
        self.__executor.spin_until_future_complete(future, timeout_sec=timeout_sec)
        return future.result()
