#  Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.

from typing import Callable, Optional

from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node

from bdai_ros2_wrappers.single_goal_multiple_action_servers import SingleGoalMultipleActionServers
from bdai_ros2_wrappers.type_hints import ActionType

# Note: for this to work correctly you must use a multi-threaded executor when spinning the node!  E.g.:
#   from rclpy.executors import MultiThreadedExecutor
#   executor = MultiThreadedExecutor()
#   rclpy.spin(node, executor=executor)


class SingleGoalActionServer(SingleGoalMultipleActionServers):
    """Wrapper around a single action server that only allows a single Action to be executing at one time. If a new
    Action.Goal is received, the existing Action (if there is one) is preemptively canceled"""

    def __init__(
        self,
        node: Node,
        action_type: ActionType,
        action_topic: str,
        execute_callback: Callable,
        callback_group: Optional[CallbackGroup] = None,
    ) -> None:
        super().__init__(
            node=node, action_server_parameters=[(action_type, action_topic, execute_callback, callback_group)]
        )
