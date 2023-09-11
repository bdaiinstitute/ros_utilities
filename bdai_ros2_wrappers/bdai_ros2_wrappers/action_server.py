# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.
from typing import Any, Optional

from rclpy.action import ActionServer
from rclpy.callback_groups import CallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.node import Node


class FriendlyActionServer(ActionServer):
    def __init__(self, node: Node, *args: Any, callback_group: Optional[CallbackGroup] = None, **kwargs: Any) -> None:
        if getattr(node, "enable_callback_isolation", False) and callback_group is None:
            callback_group = MutuallyExclusiveCallbackGroup()
        super().__init__(node, *args, callback_group=callback_group, **kwargs)
