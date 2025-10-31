# Effective synchros2

## Prefer process-wide APIs over direct node subclassing

When writing ROS 2 libraries, avoid subclassing `rclpy.node.Node` directly. Instead, use process-wide APIs to access ROS 2 resources. This avoids deadlocks common with direct node subclassing.

```python
import synchros2.process as ros_process
import synchros2.scope as ros_scope


class MySystem:
    def __init__(self) -> None:
        self.node = ros_process.ensure_node()

    def run(self):    
        ros_process.wait_for_shutdown()

@ros_process.main()
def main() -> None:
    system = MySystem()
    return system.run()

if __name__ == "__main__":
    main()
```

## Use ros_scope for libraries, ros_process for applications

Use `synchros2.scope` (`ros_scope`) to access ROS 2 resources in libraries. Use `synchros2.process` (`ros_process`) for application entrypoints. This separation ensures libraries remain reusable and decoupled from application lifecycle management.

```python
import synchros2.scope as ros_scope
from synchros2.node import Node

class SomeFeature:

    def __init__(self) -> None:
        self.node = ros_scope.load(Node, "my_node")

    def shutdown(self) -> None:
        ros_scope.unload(self.node)  # or leave it behind until process ends
```

```python
import time

import synchros2.process as ros_process
from .features import SomeFeature

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

## Subclass `synchros2.node.Node` for custom nodes

If you must subclass a node, use `synchros2.node.Node` instead of `rclpy.node.Node`. It provides a non-reentrant callback group by default, supports a `__post_init__` method for post-construction setup, and integrates with process-wide APIs for resource management.

```python
from synchros2.node import Node

class MyNode(Node):
    pass
```

## Avoid blocking in node initialization

Do not block during `__init__` of a node subclass. Blocking calls (e.g., waiting for a message or service) should be deferred to `__post_init__` or dispatched as background tasks.

```python
# Async setup in __init__
from synchros2.node import Node
from synchros2.subscription import wait_for_message_async
from std_msgs.msg import String

class ComplexInitNode(Node):
    def __init__(self, **kwargs):
        super().__init__("complex_init_node", **kwargs)
        future = wait_for_message_async(String, "foo")
        future.add_done_callback(lambda future: self.setup(future.result()))

    def setup(self, message: String) -> None:
        # do setup with message
```

```python
# Blocking in __post_init__
from synchros2.node import Node
from synchros2.subscription import wait_for_message
from std_msgs.msg import String

class PostInitNode(Node):
    def __init__(self, **kwargs):
        super().__init__("post_init_node", **kwargs)

    def __post_init__(self):
        message = wait_for_message(String, "foo")
        # do setup with message
```

## Use `create_task` to dispatch background work

Use the executors' `create_task()` method to run background tasks without blocking the main thread. This is supported by the multi-threaded executor (the default in `synchros2`), but not by single-threaded executors. Leveraging `create_task` allows efficient concurrency and integrates with ROS 2's executor model.

```python
import time
import synchros2.process as ros_process

@ros_process.main()
def main():
    node = ros_process.node()
    def background_work():
        time.sleep(5)
        node.get_logger().info("Background work done!")
    executor = ros_process.executor()
    executor.create_task(background_work)
    ros_process.wait_for_shutdown()
```

## Use single-threaded executor for time-sensitive applications

Prefer `SingleThreadedExecutor` for applications where minimal callback latency is critical. To do this, skip the default (aka `prebaked`) executor and node setup, and set custom executor and node instances. This enables advanced configuration and integration scenarios where default behavior is insufficient.

```python
from rclpy.executors import SingleThreadedExecutor
import synchros2.process as ros_process
from synchros2.node import Node

class MySystem:
    def __init__(self) -> None:
        self.node = ros_process.ensure_node()
        self.timer = self.node.create_timer(
            1.0,  # seconds
            lambda: self.node.get_logger().info("Timer triggered!")
        )

    def run(self):
        ros_process.wait_for_shutdown()

@ros_process.main(prebaked=False)
def main() -> None:
    with background(SingleThreadedExecutor()) as main.executor:
        with main.managed(Node, "my_node") as main.node:
            system = MySystem()
            return system.run()

if __name__ == "__main__":
    main()
```

## Use Python's `logging` module for general logging

Prefer the standard `logging` module for logging unless you are within a node subclass, in which case you may use the node's logger. This keeps code ROS 2 agnostic and maximizes reuse.

```python
import logging
import synchros2.process as ros_process

@ros_process.main()
def main():
    logging.info("Application started!")
```

---

By following these guidelines, you can write robust, reusable, and maintainable ROS 2 libraries and applications with `synchros2`.
