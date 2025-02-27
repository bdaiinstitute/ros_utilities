# Copyright (c) 2023 Boston Dynamics AI Institute LLC.  All rights reserved.

"""An example of a ROS 2 multi-node process using process-wide machinery.

Run with:

```sh
python3 examples/talker_listener_example.py
```
"""

import itertools
import typing

import std_msgs.msg

import synchros2.process as ros_process
from synchros2.node import Node


class TalkerNode(Node):
    """A node that publishes a phrase to a chat topic periodically."""

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
    """A node that subscribes to a chat topic and echoes it."""

    def __init__(self, **kwargs: typing.Any) -> None:
        super().__init__("listener", **kwargs)
        self.sub = self.create_subscription(std_msgs.msg.String, "chat", self.callback, 1)

    def callback(self, message: std_msgs.msg.String) -> None:
        self.get_logger().info(message.data)


def graph(**kwargs: typing.Any) -> typing.Iterable[Node]:
    return [TalkerNode(**kwargs), ListenerNode(**kwargs)]


@ros_process.main(prebaked=False, namespace=True)
def main() -> None:
    """Example entrypoint.

    It is configured almost as a regular ROS 2 aware process. That is, no process-wide node,
    no background autoscaling multi-threaded executor, and no log forwarding to the ROS 2
    logging system, but any ROS 2 nodes loaded in it will be implicitly namespaced after
    the executable basename. A convenience to better organize executables bearing many
    ROS 2 nodes.

    When executed, a graph of ROS 2 nodes is spinned in foreground, on an implicitly instantiated
    autoscaling multi-threaded executor. This continues indefinitely until the executable is
    interrupted (e.g. by a SIGINT on Ctrl + C).
    """
    ros_process.spin(graph)


if __name__ == "__main__":
    main()
