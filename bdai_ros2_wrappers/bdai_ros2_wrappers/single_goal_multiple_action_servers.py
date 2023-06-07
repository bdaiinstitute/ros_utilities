#  Copyright (c) 2023 Boston Dynamics AI Institute, Inc. All rights reserved.
import threading
from typing import Tuple, Callable, TypeVar, List, Optional

from rclpy import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import CallbackGroup

from bdai_ros2_wrappers.type_hints import Action, ActionType


class SingleGoalMultipleActionServers(object):
    def __init__(
        self, node: Node, action_server_parameters: List[Tuple[ActionType, str, Callable, Optional[CallbackGroup]]]
    ) -> None:
        self._node = node
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self._action_servers = []
        for action_type, action_topic, execute_callback, callback_group in action_server_parameters:
            self._action_servers.append(
                ActionServer(
                    node,
                    action_type,
                    action_topic,
                    execute_callback=execute_callback,
                    goal_callback=self.goal_callback,
                    handle_accepted_callback=self.handle_accepted_callback,
                    cancel_callback=self.cancel_callback,
                    callback_group=callback_group,
                )
            )

    def get_logger(self):
        return self._node.get_logger()

    def destroy(self) -> None:
        for action_server in self._action_servers:
            action_server.destroy()

    def goal_callback(self, goal: Action.Goal) -> GoalResponse:
        """Accept or reject a client request to begin an action."""
        self.get_logger().info("Received goal request")
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle: ServerGoalHandle) -> None:
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info("Aborting previous goal")
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        goal_handle.execute()

    def cancel_callback(self, cancel_request) -> CancelResponse:
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT
