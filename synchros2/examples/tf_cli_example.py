# Copyright (c) 2023 Boston Dynamics AI Institute LLC.  All rights reserved.

"""An example of a ROS 2 aware command using process-wide machinery.

Run with:

```sh
python3 examples/tf_cli_example.py
```

And follow the instructions on screen.
"""

import argparse
import math
import time
from typing import Any, Iterable, List, Optional

from geometry_msgs.msg import TransformStamped
from rclpy.executors import SingleThreadedExecutor
from tf2_ros.transform_broadcaster import TransformBroadcaster

import synchros2.process as ros_process
from synchros2.executors import background
from synchros2.node import Node
from synchros2.tf_listener_wrapper import TFListenerWrapper
from synchros2.utilities import namespace_with


class TFBroadcasterNode(Node):
    def __init__(self, tf_prefix: Optional[str] = None, **kwargs: Any) -> None:
        super().__init__("tf_broadcaster", **kwargs)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.transforms: List[TransformStamped] = []
        tree = [("world", "odom", 0.0, 1.0, math.degrees(90.0)), ("odom", "robot", 3.0, -0.5, math.degrees(-30.0))]
        for parent_frame, child_frame, x, y, theta in tree:
            transform = TransformStamped()
            transform.header.frame_id = namespace_with(tf_prefix, parent_frame)
            transform.child_frame_id = namespace_with(tf_prefix, child_frame)
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = 0.0
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = math.sin(theta / 2)
            transform.transform.rotation.w = math.cos(theta / 2)
            self.transforms.append(transform)
        self.timer = self.create_timer(1.0, self.callback)
        self.callback()  # do not wait

    def callback(self) -> None:
        stamp = self.get_clock().now().to_msg()
        for transform in self.transforms:
            transform.header.stamp = stamp
        self.tf_broadcaster.sendTransform(self.transforms)


def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--tf-prefix", type=str, default=None)
    parser.add_argument("-c", "--cache-time", type=float, default=None)
    parser.add_argument("-b", "--buffer-time", type=float, default=3.0)
    return parser


def graph(args: argparse.Namespace, **kwargs: Any) -> Iterable[Node]:
    return [TFBroadcasterNode(args.tf_prefix, **kwargs), Node("tf_prompt", **kwargs)]


def run(args: argparse.Namespace) -> None:
    print("Buffering transforms...")
    time.sleep(args.buffer_time)
    tf_listener = ros_process.tf_listener()
    assert tf_listener is not None
    print(tf_listener.buffer.all_frames_as_string())
    while True:
        target_frame = input("Please provide target frame (or press Enter to exit): ")
        if not target_frame:
            break
        source_frame = input("Please provide source frame (or press Enter to reuse): ")
        if not source_frame:
            source_frame = target_frame
        target_frame = namespace_with(args.tf_prefix, target_frame)
        source_frame = namespace_with(args.tf_prefix, source_frame)
        transform = tf_listener.lookup_a_tform_b(target_frame, source_frame, wait_for_frames=True)
        t = transform.transform.translation
        q = transform.transform.rotation
        print(f"Transform {target_frame} -> {source_frame}")
        print(f"  Translation: [x: {t.x}, y: {t.y}, z: {t.z}]")
        print(f"  Rotation: [x: {q.x}, y: {q.y}, z: {q.z}, w: {q.w}]")
        print("---")
    print("Bye bye!")


@ros_process.main(cli(), prebaked=False)
def main(args: argparse.Namespace) -> None:
    """Example entrypoint.

    It is first configured as a regular ROS 2 aware process, but process-wide
    (i.e. globally accessible) executor and node are set immediately on start.
    Still, automatic ``logging`` logs forwarding to the ROS 2 logging system is
    disabled. Implicit namespacing is also disabled. This is suitable for more
    complex use cases, that require finer control over process configuration.

    When executed, a single threaded executor is pushed to the background and
    assigned as process-wide executor. A graph of ROS 2 nodes is instantiated
    and loaded, one of which is assigned as process-wide node. These are used
    indirectly by the actual console application.
    """
    with background(SingleThreadedExecutor()) as main.executor, ros_process.managed(graph, args) as (_, main.node):
        main.tf_listener = TFListenerWrapper(main.node, cache_time_s=args.cache_time)
        run(args)


if __name__ == "__main__":
    main()
