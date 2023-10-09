# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.

"""
An example of a ROS 2 single node process using process-wide machinery.

Run with:

```sh
python3 examples/talker_example.py chit chat
```

You can speed up publication with:

```sh
python3 examples/talker_example.py chit chat --ros-args -p rate:=5.0  # Hz
```
"""

import itertools
import typing

import bdai_ros2_wrappers.process as ros_process
from bdai_ros2_wrappers.node import Node


class TalkerNode(Node):
    """A node that logs the same phrase periodically."""

    def __init__(self, phrase: str, **kwargs: typing.Any) -> None:
        super().__init__("talker", **kwargs)
        self.phrase = phrase
        self.counter = itertools.count(start=1)
        rate = self.declare_parameter("rate", 1.0).value
        self.timer = self.create_timer(1 / rate, self.callback)

    def callback(self) -> None:
        self.get_logger().info(f"{self.phrase} (#{next(self.counter)})")


@ros_process.main(prebaked=False)
def main(args: typing.Sequence[str]) -> None:
    ros_process.spin(TalkerNode, " ".join(args[1:]))


if __name__ == "__main__":
    main()
