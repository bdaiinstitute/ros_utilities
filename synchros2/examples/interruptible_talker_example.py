# Copyright (c) 2024 Boston Dynamics AI Institute LLC.  All rights reserved.

"""An example of an interruptible ROS 2 single node process using process-wide machinery.

Run with:

```sh
python3 examples/interruptible_talker_example.py chit chat
```

You can speed up publication with:

```sh
python3 examples/interruptible_talker_example.py chit chat --ros-args -p rate:=5.0  # Hz
```

On Ctrl + C, the node will be interrupted. Another Ctrl + C will terminate the example
and one last "goodbye" message will be published by the talker.
"""

import itertools
import typing

import std_msgs.msg
from rclpy.executors import SingleThreadedExecutor

import synchros2.process as ros_process
from synchros2.executors import foreground
from synchros2.node import Node


class InterruptibleTalkerNode(Node):
    """A node that logs the same phrase periodically."""

    def __init__(self, phrase: str, **kwargs: typing.Any) -> None:
        super().__init__("interruptible_talker", **kwargs)
        self.phrase = phrase
        self.counter = itertools.count(start=1)
        rate = self.declare_parameter("rate", 1.0).value
        self.pub = self.create_publisher(std_msgs.msg.String, "chat", 1)
        self.timer = self.create_timer(1 / rate, self.callback)

    def callback(self) -> None:
        message = f"{self.phrase} (#{next(self.counter)})"
        self.pub.publish(std_msgs.msg.String(data=message))
        self.get_logger().info(message)

    def destroy_node(self) -> None:
        message = "Goodbye!"
        self.pub.publish(std_msgs.msg.String(data=message))
        self.get_logger().info(message)
        super().destroy_node()


@ros_process.main(prebaked=False, interruptible=True)
def main(args: typing.Sequence[str]) -> None:
    """Example entrypoint, taking command-line arguments.

    It is configured as ROS 2 aware process. That is, no process-wide node,
    no background autoscaling multi-threaded executor, no log forwarding to the ROS 2
    logging system, and no implicit node namespacing. In other words, a run-off-the-mill
    executable that uses ROS 2. Well, almost.

    When executed, a single `InterruptibleTalkerNode` is instantiated and spinned in the
    foreground. This will continue indefinitely until the executable is interrupted
    (e.g. by a SIGINT on Ctrl + C). Another Ctrl + C will confirm the interruption.
    Any other key will resume execution.
    """
    with foreground(SingleThreadedExecutor()) as main.executor:  # noqa: SIM117
        with main.managed(InterruptibleTalkerNode, " ".join(args[1:])) as main.node:
            while True:
                try:
                    main.executor.spin()
                except KeyboardInterrupt:
                    input(
                        "Press Ctrl + C again to confirm, or press any other key to continue",
                    )


if __name__ == "__main__":
    main()
