# Process-wide APIs

Process-wide APIs in `synchros2` wrap [`rclpy`](https://github.com/ros2/rclpy) to provide a simpler, more intuitive experience for common ROS 2 use cases. 

## Nodes v. Processes

In ROS 2, a **node** is a fundamental unit of computation that communicates with other nodes using topics, services, and actions. Each node typically runs within an OS process, but a single process can host multiple nodes. This differs from ROS 1, where each node usually maps directly to a separate OS process. In ROS 2, nodes are more lightweight and can be managed together within the same process, allowing for more flexible resource sharing and easier composition.

## ROS 2 Awareness for Processes

Process-wide APIs are built around the concept of a ROS 2 aware **scope**, which manages the lifetime of a thread-local graph of ROS 2 nodes and an executor that dispatches work for them. Scopes can be nested, enforcing locality of ROS 2 usage, and the innermost scope is always accessible via `synchros2.scope` APIs.

A scope may also be process-local, introducing the notion of a ROS 2 aware **process**. Only one process may be active as a top-level scope at any time. The current process and its scope are always accessible process-wide via `synchros2.process` APIs.

This design enables process-wide (and thread-wide) access to locally managed ROS 2 entities, with well-defined ownership and lifetime. Callbacks are dispatched in the background by default, supporting both synchronous and asynchronous programming out-of-the-box.

These APIs are especially useful for users familiar with [`rospy`](http://wiki.ros.org/rospy), making the transition to ROS 2 smoother.

## Setting up Single Node Processes

To use ROS 2 without boilerplate, decorate your executable entrypoint (or `main` function) with `synchros2.process.main`. This automatically sets up a process-wide node and executor, and handles initialization and shutdown for you:

```python
import logging
import time
import synchros2.process as ros_process
import std_msgs.msg

@ros_process.main()
def entrypoint() -> None:
    # Initialization is automatic
    node = ros_process.node()  # or entrypoint.node
    assert node is not None
    pub = node.create_publisher(std_msgs.msg.String, "test", 1)

    def callback() -> None:
        time.sleep(10)  # Callbacks can block safely
        pub.publish(std_msgs.msg.String(data="testing"))

    executor = ros_process.executor()  # or entrypoint.executor
    assert executor is not None
    executor.create_task(callback)  # Dispatch callback for execution

    time.sleep(10)  # Main thread can block too

    node.get_logger().info("testing")
    logging.info("testing")  # Python logging is integrated

    try:
        ros_process.wait_for_shutdown()  # Wait for Ctrl+C
    except KeyboardInterrupt:
        pass  # Avoid traceback printing
    # Cleanup and shutdown are automatic

if __name__ == "__main__":
    entrypoint()
```

The process-wide node and executor are accessible via `synchros2.process.node()` and `synchros2.process.executor()`. This pattern is ideal for quick prototyping and simple scripts, as you can make blocking calls from anywhere.

## Setting up Multi-Node Processes

You can spin up multiple ROS 2 nodes and skip the default process-wide node if unnecessary. For example, to load and spin two nodes:

```python
import logging
import time
from typing import Any, List
import synchros2.process as ros_process
from synchros2.node import Node

def graph(**kwargs: Any) -> List[Node]:
    # Forward all kwargs for proper node loading
    return [Node("my_node", **kwargs), Node("my_other_node", **kwargs)]

@ros_process.main(prebaked=False)
def main() -> None:
    ros_process.spin(graph)  # or main.spin (blocks until shutdown)

if __name__ == "__main__":
    main()
```

You can use node factories to load or spin collections of nodes at once. The `prebaked=False` argument disables the default node and executor, giving you full control.

## Interactive Multi-Node Applications

For interactive applications, foreground spinning is usually desirable. Here’s an example that waits for user input, sleeps, and echoes the input from a callback without deadlocking:

```python
import logging
import time
from typing import Any, List
from synchros2.futures import wait_for_future
from synchros2.node import Node
import synchros2.process as ros_process

def graph(**kwargs: Any) -> List[Node]:
    return [Node("my_node", **kwargs), Node("my_other_node", **kwargs)]

def work(prompt: str) -> None:
    def worker() -> None:
        node = ros_process.node()
        assert node is not None
        time.sleep(5.0)  # Sleeping is safe!
        node.get_logger().info(prompt + " done!")
    executor = ros_process.executor()
    assert executor is not None
    future = executor.create_task(worker)
    wait_for_future(future)  # Block until worker is done

@ros_process.main()
def main() -> None:
    ros_process.load(graph)
    while True:
        work(input())

if __name__ == "__main__":
    main()
```

**Caution:** Using `input()` for interactive CLI disables running with `launch`, as `launch` does not pipe its own process `stdin` to subprocesses.

## Command-Line Single Node Applications

You can use command-line arguments to configure ROS 2 processes. For example:

```python
import argparse
import logging
import synchros2.process as ros_process
import synchros2.scope as ros_scope

class Application:
    def __init__(self, robot_name: str) -> None:
        self.robot_name = robot_name
        self.node = ros_scope.ensure_node()  # raises if node is not setup

def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("robot_name")
    parser.add_argument("-v", "--verbose", action="store_true")
    # Define process arguments based on CLI
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

Process arguments are set via CLI, allowing fine-grained control over configuration (e.g., custom main node, executor, etc.).
