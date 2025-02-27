# Copyright (c) 2023 Boston Dynamics AI Institute LLC.  All rights reserved.

"""An example of `logging` logs forward to the ROS 2 logging system.

Run with:

```sh
python3 examples/logs_to_ros_example.py
```
"""

import itertools
import logging

import rclpy

from synchros2.logging import logs_to_ros


def main() -> None:
    rclpy.init()
    try:
        node = rclpy.create_node("example_logger")
        counter = itertools.count(start=1)

        def callback() -> None:
            logging.info("Called back %d times", next(counter))

        node.create_timer(1, callback)
        with logs_to_ros(node):
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    root = logging.getLogger()
    root.setLevel(logging.INFO)
    main()
