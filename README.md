# ros_utilities
Wrappers and other utilities for ROS2

## Getting started

### Process-wide APIs

These APIs wrap [`rclpy`](https://github.com/ros2/rclpy) APIs to provide a simpler UX in common use cases.
All APIs are thread-safe and exception-safe. Resource management is automatic. By default, callbacks are
dispatched in the background, enabling both synchronous and asynchronous programming.

These APIs are also quite handy to reconcile past [`rospy`](http://wiki.ros.org/rospy) experience with ROS 2.

#### Setting up single node processes

Just decorate your executable entrypoint (or `main` function) with `bdai_ros2_wrappers.process.main`:

```python
import logging
import time

import bdai_ros2_wrappers.process as ros_process

import std_msgs.msg

@ros_process.main()
def entrypoint() -> None:
    # no need to initialize, it is automatic
    node = ros_process.node()  # or entrypoint.node
    assert node is not None
    pub = node.create_publisher(std_msgs.msg.String, "test", 1)

    def callback() -> None:
        time.sleep(10)  # you can block in a callback
        pub.publish(std_msgs.msg.String(data="testing"))
    executor = ros_process.executor()  # or entrypoint.executor
    assert executor is not None
    executor.create_task(callback)

    time.sleep(10)  # you can block in the main thread

    node.get_logger().info("testing")
    logging.info("testing")  # you can use Python logging

    return  # no need to cleanup or shutdown, it is automatic

if __name__ == "__main__":
    entrypoint()
```

This is ideal for quick prototyping and simple scripts.

#### Setting up multi-node processes

You can spin as many ROS 2 nodes as you need, and skip the default process-wide node if unnecessary:

```python
import logging
import time

from typing import Any, List

import bdai_ros2_wrappers.process as ros_process
from bdai_ros2_wrappers.node import Node

def graph(**kwargs: Any) -> List[Node]:
    # make sure to forward all keyword arguments for proper loading!
    return [Node("my_node", **kwargs), Node("my_other_node", **kwargs)]

@ros_process.main(prebaked=False)
def main() -> None:
    ros_process.load(Node, "_hidden_node")  # or main.load
    ros_process.spin(graph)  # or main.spin, and it will block!

if __name__ == "__main__":
    main()
```

Note the use of ROS 2 node(s) factories, to load (or spin) entire collections at once.

#### Setting up interactive multi-node applications

For interactive applications, you will most likely want to keep automatic spinning in place:

```python
import logging
import time

from typing import Any, List

from bdai_ros2_wrappers.futures import wait_for_future
from bdai_ros2_wrappers.node import Node
import bdai_ros2_wrappers.process as ros_process

def graph(**kwargs: Any) -> List[Node]:
    # make sure to forward all keyword arguments for proper loading!
    return [Node("my_node", **kwargs), Node("my_other_node", **kwargs)]

def work(prompt: str) -> None:
    def worker() -> None:
        node = ros_process.node()
        assert node is not None
        time.sleep(5.0)  # sleeping is safe!
        node.get_logger().info(prompt + " done!")
    executor = ros_process.executor()
    assert executor is not None
    future = executor.create_task(worker)  # dispatch worker to executor
    wait_for_future(future)  # block until worker is done

@ros_process.main()
def main() -> None:
    ros_process.load(graph)  # or main.load
    while True:
        work(input())

if __name__ == "__main__":
    main()
```

#### Setting up command-line single node applications

```python
import argparse
import logging
import time

from typing import Any

from bdai_ros2_wrappers.futures import wait_for_future
from bdai_ros2_wrappers.node import Node
import bdai_ros2_wrappers.process as ros_process

class Application(Node):

    def __init__(self, robot_name: str, **kwargs: Any) -> None:
        super().__init__("app_node", **kwargs)
        self.robot_name = robot_name

def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("robot_name")
    parser.add_argument("-v", "--verbose", action="store_true")
    # define ROS 2 aware process arguments based on CLI arguments
    parser.set_default(process_args=lambda args: dict(
        prebaked=False, forward_logging=args.verbose))
    return parser

@ros_process.main(cli())
def main(args: argparse.Namespace) -> None:
    # set process-wide node for log forwarding to take effect
    with ros_process.managed(Application, args.robot_name) as main.node:
        if args.verbose:
            root = logging.getLogger()
            root.setLevel(logging.INFO)
        logging.info("Application started!")
        ros_process.spin()  # or main.spin

if __name__ == "__main__":
    main()
```

Note that process arguments are set via CLI. Also, note that a process-wide node is set explicitly (rather than implicitly). 
The same can be done for process-wide executors. This allows fine grained control over ROS 2 configuration.

#### Writing ROS 2 aware libraries

You can use process-wide APIs to fetch ROS 2 defaults deep down call hierarchies, when passing nodes explicitly adds unnecessary clutter:

```python
import time

import bdai_ros2_wrappers.process as ros_process
import bdai_ros2_wrappers.scope as ros_scope
from bdai_ros2_wrappers.node import Node

class SomeFeature:

    def __init__(self) -> None:
        self.node = ros_scope.load(Node, "my_node")

    def shutdown(self) -> None:
        ros_scope.unload(self.node)  # or leave it behind until process ends

@ros_process.main()
def main() -> None:
    feature = SomeFeature()
    try:
        time.sleep(10.0)
    finally:
        feature.shutdown()

if __name__ == "__main__":
    main()
```

Note the use of `ros_scope` instead of `ros_process`. Both offer the same APIs, but the former reaches out to the innermost scope. This allows the library to work in more complex, multi-threaded, multi-scope applications, and for testing cases.

#### Writing ROS 2 aware tests

To write tests, setup a global ROS 2 aware scope instead of a process.
Consider using namespaces to avoid default, hidden ROS 2 names.

Using [`pytest`](https://docs.pytest.org/en/6.2.x/contents.html) (recommended):

```python
from typing import Iterable

import bdai_ros2_wrappers.scope as ros_scope
from bdai_ros2_wrappers.scope import ROSAwareScope

import pytest

@pytest.fixture
def ros() -> Iterable[ROSAwareScope]:
    with ros_scope.top(global_=True, namespace="fixture") as top:
        yield top

def test_it(ros: ROSAwareScope) -> None:
    assert ros.node is not None
    ros.node.get_logger().info("Logging!")
```

Using [`unittest`](https://docs.python.org/3/library/unittest.html):

```python
import unittest

import bdai_ros2_wrappers.scope as ros_scope

class TestCase(unittest.TestCase):

    def test_it(self) -> None:
        with ros_scope.top(global_=True, namespace="fixture") as ros:
            self.assertIsNotNone(ros.node)
            ros.node.get_logger().info("Logging!")
```

## Contribution
To contribute, install `pre-commit` via pip, run `pre-commit install` and then run `pre-commit run --all-files` to
verify that your code will pass inspection.
```bash
git clone https://github.com/bdaiinstitute/ros_utilities.git
cd ros_utilities
pip3 install pre-commit
pre-commit install
pre-commit run --all-files
```

Now whenever you commit code to this repository, it will be checked against our `pre-commit` hooks. You can also run
`git commit --no-verify` if you wish you commit without checking against the hooks.
