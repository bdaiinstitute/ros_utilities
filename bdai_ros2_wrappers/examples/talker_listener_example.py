# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.

"""
An example of a ROS 2 multi-node process using process-wide machinery.

Run with:

```sh
python3 examples/talker_listener_example.py
```
"""

import itertools
import typing

import std_msgs.msg

import bdai_ros2_wrappers.process as ros_process
from bdai_ros2_wrappers.node import Node


class TalkerNode(Node):
    """A node that publishes a phrase periodically."""

    def __init__(self, **kwargs: typing.Any) -> None:
        super().__init__("talker", **kwargs)
        self.counter = itertools.count(start=1)
        self.pub = self.create_publisher(std_msgs.msg.String, "chat", 1)
        rate = self.declare_parameter("rate", 1.0).value
        self.timer = self.create_timer(1 / rate, self.callback)

    def callback(self) -> None:
        message = std_msgs.msg.String(data=f"Hi there, from {self.get_name()} (#{next(self.counter)})")
        self.pub.publish(message)


class ListenerNode(Node):
    """A node that subscribes to chattering."""

    def __init__(self, **kwargs: typing.Any) -> None:
        super().__init__("listener", **kwargs)
        self.sub = self.create_subscription(std_msgs.msg.String, "chat", self.callback, 1)

    def callback(self, message: std_msgs.msg.String) -> None:
        self.get_logger().info(message.data)


def graph(**kwargs: typing.Any) -> typing.Iterable[Node]:
    return [TalkerNode(**kwargs), ListenerNode(**kwargs)]


@ros_process.main(prebaked=False, namespace=True)
def main() -> None:
    ros_process.spin(graph)


if __name__ == "__main__":
    main()
