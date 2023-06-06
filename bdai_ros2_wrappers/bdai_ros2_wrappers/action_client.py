# Copyright [2023] Boston Dynamics AI Institute, Inc.
from typing import Optional

import rclpy.action

from bdai_ros2_wrappers.node import NodeWrapper

from rclpy import Context


class ActionClientWrapper(rclpy.action.ActionClient):
    def __init__(
        self,
        action_type,
        action_name: str,
        node_name: str,
        namespace: Optional[str] = None,
        context: Optional[Context] = None,
    ):
        self._node_wrapper = NodeWrapper(
            f"{node_name}_{action_name}_client_wrapper_node", namespace=namespace, context=context
        )
        super().__init__(self._node_wrapper.node, action_type, action_name)
        self._node_wrapper.node.get_logger().info(
            "Waiting for action server for " + self._node.get_namespace() + "/" + action_name
        )
        self.wait_for_server()
        self._node_wrapper.node.get_logger().info("Found server")

    def send_goal_and_wait(self, action_goal, timeout_sec=None):
        future = self.send_goal_async(action_goal)
        goal_handle = self._node_wrapper.spin_until_future_complete(future, timeout_sec=timeout_sec)

        if not goal_handle.accepted:
            self._node_wrapper.node.get_logger().error("Goal was not accepted")
            return None

        future = goal_handle.get_result_async()
        return self._node_wrapper.spin_until_future_complete(future, timeout_sec=timeout_sec)
