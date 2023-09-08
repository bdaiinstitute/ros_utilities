# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.
from typing import Optional

from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import CallbackGroup
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class FriendlyActionServer(ActionServer):

    def __init__(
        self,
        node: Node, *args,
        callback_group: Optional[CallbackGroup] = None, **kwargs
    ) -> None:
        if getattr(node, 'enable_callback_isolation', False) and callback_group is None:
            callback_group = MutuallyExclusiveCallbackGroup()
        super().__init__(node, *args, callback_group=callback_group, **kwargs)
