# Copyright (c) 2023 Boston Dynamics AI Institute LLC.  All rights reserved.
import threading
from typing import Any, Callable, List, Optional, Tuple

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import GoalEvent, RCLError, ServerGoalHandle
from rclpy.callback_groups import CallbackGroup
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.node import Node

from synchros2.type_hints import ActionType
from synchros2.utilities import synchronized


class SingleGoalMultipleActionServers:
    """Wrapper around multiple action servers that only allows a single Action to be executing at one time.

    If a new Action.Goal is received by any of the action servers, the existing Action (if there is one) is preemptively
    canceled.
    """

    def __init__(
        self,
        node: Node,
        action_server_parameters: List[Tuple[ActionType, str, Callable, Optional[CallbackGroup]]],
        nosync: bool = False,
    ) -> None:
        """Constructor.

        Args:
            node: ROS 2 node to use for action servers.
            action_server_parameters: tuples per action server, listing action type, action name,
            action execution callback, and action callback group (which may be None).
            nosync: whether to synchronize action execution callbacks using locks or not.
            Set to True when action execution callback already enforce mutual exclusion.
        """
        self._node = node
        self._goal_handle_lock = threading.Lock()
        self._goal_handle: Optional[ServerGoalHandle] = None
        if not nosync:
            execution_lock = threading.Lock()
            action_server_parameters = [
                (action_type, action_topic, synchronized(execute_callback, execution_lock), callback_group)
                for action_type, action_topic, execute_callback, callback_group in action_server_parameters
            ]
        self._action_servers = [
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
            for action_type, action_topic, execute_callback, callback_group in action_server_parameters
        ]

    def get_logger(self) -> RcutilsLogger:
        """Returns the ros logger"""
        return self._node.get_logger()

    def destroy(self) -> None:
        """Destroy all of the internal action servers"""
        for action_server in self._action_servers:
            action_server.destroy()

    def goal_callback(self, goal: Any) -> GoalResponse:
        """Accept or reject a client request to begin an action."""
        self.get_logger().info("Received goal request")
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle: ServerGoalHandle) -> None:
        """Callback triggered when an action is accepted."""
        with self._goal_handle_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None:
                self.get_logger().info("Canceling previous goal")
                try:
                    self._goal_handle._update_state(GoalEvent.CANCEL_GOAL)
                except RCLError as ex:
                    self.get_logger().debug(f"Failed to cancel goal: {ex}")
            self._goal_handle = goal_handle

        goal_handle.execute()

    def cancel_callback(self, cancel_request: Any) -> CancelResponse:
        """Accept or reject a client's request to cancel an action."""
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT
