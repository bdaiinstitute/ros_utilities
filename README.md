# ros_utilities
Wrappers and other utilities for ROS2

## Getting started

### Process-wide APIs

These APIs wrap [`rclpy`](https://github.com/ros2/rclpy) APIs to provide a simpler UX in common use cases.

These APIs are built around the notion of a ROS 2 aware _scope_. ROS 2 aware scopes manage the lifetime of a
thread local graph of ROS 2 nodes, along with an executor that dispatches work for them. ROS 2 nodes may be
loaded and unloaded (i.e. instantiated and put to spin, and explicitly destroyed, respectively) or have their
entire lifecycle be managed (i.e. bound to a context manager). A ROS 2 aware scope may also define a main
ROS 2 node (for ease of use, for log forwarding, etc.). ROS 2 aware scopes may be nested, enforcing locality
of ROS 2 usage in a codebase, though the innermost scope is always accessible through `bdai_ros2_wrappers.scope`
module-level APIs.

A ROS 2 aware scope may also be process local (i.e. global within a process), which allows for the notion
of a ROS 2 aware _process_. Only one ROS 2 aware process may be active at any point in time as a top-level
scope i.e. a ROS 2 aware process may not start within an existing ROS 2 aware scope. The current ROS 2 aware
process and associated scope are always accessible process-wide through `bdai_ros2_wrappers.process`
module-level APIs.

These notions afford process-wide (and thread-wide) access to locally managed ROS 2 entities and thus ownership
and lifetime is well defined. Moreover, callbacks are dispatched in the background by default, enabling both
synchronous and asynchronous programming out-of-the-box.

These APIs are also quite handy to reconcile past [`rospy`](http://wiki.ros.org/rospy) experience with ROS 2.

#### Setting up single node processes

To make use of ROS 2 without getting into the details, just decorate your executable entrypoint (or `main` function)
with `bdai_ros2_wrappers.process.main`:

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
    executor.create_task(callback)  # dispatch callback for execution

    time.sleep(10)  # you can block in the main thread

    node.get_logger().info("testing")
    logging.info("testing")  # you can use Python logging

    return  # no need to cleanup or shutdown, it is automatic

if __name__ == "__main__":
    entrypoint()
```

Note a ROS 2 node and an executor are accessible process-wide through `bdai_ros2_wrappers.process` module APIs.
This is ideal for quick prototyping and simple scripts, as the UX is largely intuitive e.g. you can make blocking
calls from virtually anywhere.

#### Setting up multi-node processes

You can spin as many ROS 2 nodes as you need, and skip the default process-wide node if unnecessary. Here's an example that
loads three (3) ROS 2 nodes and spins them indefinitely:

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

For interactive applications, you will most likely want to keep automatic spinning in place. Here's an example that waits
for user input, sleeps for 5 seconds, and then echoes that user input from a callback without deadlocking:

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

Note the use of `input` to read from `stdin`. This effectively prevents this executable from running with `launch`,
as `launch` does **not** pipe its own process `stdin` to that of any executed subprocess.

#### Setting up command-line single node applications

Command-line arguments may affect ROS 2 configuration if need be:

```python
import argparse
import logging

import bdai_ros2_wrappers.process as ros_process
import bdai_ros2_wrappers.scope as ros_scope

class Application:

    def __init__(self, robot_name: str) -> None:
        self.robot_name = robot_name
        self.node = ros_scope.node()

def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("robot_name")
    parser.add_argument("-v", "--verbose", action="store_true")
    # define ROS 2 aware process arguments based on CLI arguments
    parser.set_defaults(process_args=lambda args: dict(forward_logging=args.verbose))
    return parser

@ros_process.main(cli(), autospin=False)
def main(args: argparse.Namespace) -> None:
    app = Application(args.robot_name)
    if args.verbose:
        root = logging.getLogger()
        root.setLevel(logging.INFO)
    logging.info("Application started!")
    ros_process.spin()  # or main.spin

if __name__ == "__main__":
    main()
```

Note that process arguments are set via CLI. Also, note that a process-wide node is set explicitly (rather than implicitly).
The same can be done for process-wide executors. This allows fine grained control over ROS 2 configuration, which is useful
to those with special needs e.g. a custom main node (like in this case), a custom executor, etc.

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

Note the use of `ros_scope` instead of `ros_process`. Both offer roughly the same APIs, but the former reaches out to the innermost scope,
whereas the latter presumes a process is active. Therefore, and as a rule of thumb, libraries should always use `bdai_ros2_wrappers.scope` APIs.
This will allow such libraries to work in more complex, multi-threaded, multi-scope applications, and simplify testing.

#### Writing ROS 2 aware tests

To write tests, set up a global ROS 2 aware scope instead of a process. Also, consider using namespaces to avoid default, hidden ROS 2 names.

Using [`pytest`](https://docs.pytest.org/en/6.2.x/contents.html) (recommended):

```python
from typing import Iterator

import bdai_ros2_wrappers.scope as ros_scope
from bdai_ros2_wrappers.scope import ROSAwareScope

import pytest

@pytest.fixture
def ros() -> Iterator[ROSAwareScope]:
    """
    A pytest fixture that will set up and yield a ROS 2 aware global scope to each test that requests it.

    See https://docs.pytest.org/en/6.2.x/fixture.html for a primer on pytest fixtures.
    """
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

#### Reference

Check the [examples](./examples) available or read up `bdai_ros2_wrappers` API documentation.

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
