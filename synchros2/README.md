# `synchros2`

At its core, `synchros2` is nothing but a collection of utilities and wrappers built on top of [`rclpy`](https://github.com/ros2/rclpy). When used in concert, these utilities and wrappers simplify ROS 2 usage by enabling standard, idiomatic, synchronous Python programming. To that end, `synchros2` relies on heavy yet implicit concurrency and thus there is overhead in its simplicity.

## Table of contents

- [Features](#features)
  - [Process-wide APIs](#process-wide-apis)
  - [Actionable and serviced APIs](#actionable-and-serviced-apis)
  - [Message feed APIs](#message-feed-apis)
- [Guidelines](#guidelines)
  - [Integration testing](#integration-testing)

## Features

### Process-wide APIs

Process-wide APIs are built around the notion of a ROS 2 aware _scope_. ROS 2 aware scopes manage the lifetime of a
thread local graph of ROS 2 nodes, along with an executor that dispatches work for them. ROS 2 nodes may be
loaded and unloaded (i.e. instantiated and put to spin, and explicitly destroyed, respectively) or have their
entire lifecycle be managed (i.e. bound to a context manager). A ROS 2 aware scope may also define a main
ROS 2 node (for ease of use, for log forwarding, etc.). ROS 2 aware scopes may be nested, enforcing locality
of ROS 2 usage in a codebase, though the innermost scope is always accessible through `synchros2.scope`
module-level APIs.

A ROS 2 aware scope may also be process local (i.e. global within a process), which allows for the notion
of a ROS 2 aware _process_. Only one ROS 2 aware process may be active at any point in time as a top-level
scope i.e. a ROS 2 aware process may not start within an existing ROS 2 aware scope. The current ROS 2 aware
process and associated scope are always accessible process-wide through `synchros2.process`
module-level APIs.

These notions afford process-wide (and thread-wide) access to locally managed ROS 2 entities and thus ownership
and lifetime is well defined. Moreover, callbacks are dispatched in the background by default, enabling both
synchronous and asynchronous programming out-of-the-box.

These APIs are also quite handy to reconcile past [`rospy`](http://wiki.ros.org/rospy) experience with ROS 2.

#### Common use cases

##### Setting up single node processes

To make use of ROS 2 without getting into the details, just decorate your executable entrypoint (or `main` function)
with `synchros2.process.main`:

```python
import logging
import time

import synchros2.process as ros_process

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

    try: 
        ros_process.wait_for_shutdown()  # you can wait for Ctrl + C
    except KeyboardInterrupt:
        pass  # to avoid traceback printing

    return  # no need to cleanup or shutdown, it is automatic

if __name__ == "__main__":
    entrypoint()
```

Note a ROS 2 node and an executor are accessible process-wide through `synchros2.process` module APIs.
This is ideal for quick prototyping and simple scripts, as the UX is largely intuitive e.g. you can make blocking
calls from virtually anywhere.

##### Setting up multi-node processes

You can spin as many ROS 2 nodes as you need, and skip the default process-wide node if unnecessary. Here's an example that
loads three (3) ROS 2 nodes and spins them indefinitely:

```python
import logging
import time

from typing import Any, List

import synchros2.process as ros_process
from synchros2.node import Node

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

##### Setting up interactive multi-node applications

For interactive applications, you will most likely want to keep automatic spinning in place. Here's an example that waits
for user input, sleeps for 5 seconds, and then echoes that user input from a callback without deadlocking:

```python
import logging
import time

from typing import Any, List

from synchros2.futures import wait_for_future
from synchros2.node import Node
import synchros2.process as ros_process

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

##### Setting up command-line single node applications

Command-line arguments may affect ROS 2 configuration if need be:

```python
import argparse
import logging

import synchros2.process as ros_process
import synchros2.scope as ros_scope

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

##### Writing ROS 2 aware libraries

You can use process-wide APIs to fetch ROS 2 defaults deep down call hierarchies, when passing nodes explicitly adds unnecessary clutter:

```python
import time

import synchros2.process as ros_process
import synchros2.scope as ros_scope
from synchros2.node import Node

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
whereas the latter presumes a process is active. Therefore, and as a rule of thumb, libraries should always use `synchros2.scope` APIs.
This will allow such libraries to work in more complex, multi-threaded, multi-scope applications, and simplify testing.

##### Writing ROS 2 aware tests

To write tests, set up a global ROS 2 aware scope instead of a process. Also, consider using namespaces to avoid default, hidden ROS 2 names.

Using [`pytest`](https://docs.pytest.org/en/6.2.x/contents.html) (recommended):

```python
from typing import Iterator

import synchros2.scope as ros_scope
from synchros2.scope import ROSAwareScope

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

import synchros2.scope as ros_scope

class TestCase(unittest.TestCase):

    def test_it(self) -> None:
        with ros_scope.top(global_=True, namespace="fixture") as ros:
            self.assertIsNotNone(ros.node)
            ros.node.get_logger().info("Logging!")
```

### Actionable and serviced APIs

These APIs wrap those in [`rclpy.client`](https://github.com/ros2/rclpy/tree/rolling/rclpy/rclpy/client.py) and [`rclpy.action.client`](https://github.com/ros2/rclpy/tree/rolling/rclpy/rclpy/action/client.py) to provide a simpler UX when dealing with ROS 2 actions and services.

Actionable and serviced APIs abstract ROS 2 action and service calls behind an interface that resembles that of remote procedure calls. These can be invoked either synchronously or asynchronously. When used asynchronously, serviced APIs return plain futures whereas actionable APIs return action futures. Action futures build on the notion of a future to track actions' feedback, status, and result.

Both abstractions are well integrated with [ROS 2 aware scopes and processes](#process-wide-apis).

#### Common use cases

The following snippets make use of standard ROS 2 [`examples`](https://index.ros.org/r/examples/github-ros2-examples) and [`example_interfaces`](https://index.ros.org/p/example_interfaces).

##### Invoking a service synchronously

You can use a serviced API as you would use any other callable:

```python
import argparse

from example_interfaces.srv import AddTwoInts

from synchros2.service import Serviced, ServiceTimeout, ServiceError
import synchros2.process as ros_process

def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("a", type=int)
    parser.add_argument("b", type=int)
    return parser

@ros_process.main(cli())
def main(args: argparse.Namespace) -> None:
    add_two_ints = Serviced(AddTwoInts, "add_two_ints")
    if not add_two_ints.wait_for_service(timeout_sec=5.0):
        print(f"No {add_two_ints.service_name} services found")
        return
    try:
        print(f"Computing {args.a} + {args.b}...")
        result = add_two_ints(AddTwoInts.Request(a=args.a, b=args.b), timeout_sec=5.0)
        print("Result is", result.sum)
    except ServiceTimeout:
        print("Computation timed out")
    except ServiceError as e:
        print(f"Computation failed: {e}")

if __name__ == "__main__":
    main()
```

**Note**: you may use servers in the `examples_rclpy_minimal_service` package to test this.

Serviced API calls are synchronous by default. This can also be made explicit by calling `synchronously()` on them instead e.g. `add_two_ints.synchronously()`.
All service outcomes other than nominal success are signaled using exceptions. An optional timeout prevents calling (and blocking on) a service request indefinitely.

##### Invoking a service asynchronously

You can get a future service response instead of blocking on call too:

```python
import argparse

from example_interfaces.srv import AddTwoInts

from synchros2.service import Serviced
from synchros2.futures import wait_for_future
import synchros2.process as ros_process

def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("a", type=int)
    parser.add_argument("b", type=int)
    return parser

@ros_process.main(cli())
def main(args: argparse.Namespace) -> None:
    add_two_ints = Serviced(AddTwoInts, "add_two_ints")
    if not add_two_ints.wait_for_service(timeout_sec=5.0):
        print(f"No {add_two_ints.service_name} services found")
        return
    print(f"Computing {args.a} + {args.b}...")
    future = add_two_ints.asynchronously(AddTwoInts.Request(a=args.a, b=args.b))
    if not wait_for_future(future, timeout_sec=5.0):
        print("Computation did not complete in time")
        future.cancel()
        return
    result = future.result()
    print("Result is", result.sum)

if __name__ == "__main__":
    main()
```

**Note**: you may use servers in the `examples_rclpy_minimal_service` package to test this.

Service response must be waited on, either explicitly and with a timeout or implicitly by early result request.
Note fetching the future call result may raise.

##### Invoking an action synchronously

You can use an actionable API as you would use any other callable:

```python
import argparse

from example_interfaces.action import Fibonacci

from synchros2.action import Actionable
from synchros2.action import (
    ActionTimeout, ActionRejected, ActionCancelled, ActionAborted
)
import synchros2.process as ros_process

def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("order", type=int)
    return parser

@ros_process.main(cli())
def main(args: argparse.Namespace) -> None:
    compute_fibonacci_sequence = Actionable(Fibonacci, "fibonacci")
    if not compute_fibonacci_sequence.wait_for_server(timeout_sec=5.0):
        print(f"No {compute_fibonacci_sequence.action_name} action server found")
        return
    try:
        print(f"Computing Fibonacci sequence for order N = {args.order}...")
        result = compute_fibonacci_sequence(Fibonacci.Goal(order=args.order), timeout_sec=5.0)
        print("Sequence is", result.sequence)
    except ActionRejected:
        print("Computation rejected")
    except ActionTimeout:
        print("Computation timed out")
    except ActionAborted:
        print("Computation aborted")
    except ActionCancelled:
        print("Computation cancelled")

if __name__ == "__main__":
    main()
```

**Note**: you may use servers in the `examples_rclpy_minimal_action_server` package to test this.

Actionable API calls are synchronous by default. This can also be made explicit by calling `synchronously()` on them instead e.g. `compute_fibonacci_sequence.synchronously()`.
All action outcomes other than nominal success are signaled using exceptions. Action feedback is ignored unless a callback is specified on call.
An optional timeout prevents pursuing (and blocking on) an action indefinitely.

##### Invoking an action asynchronously

You can get a future to an ongoing action instead of blocking on it:

```python
import argparse

from example_interfaces.action import Fibonacci

from synchros2.action import Actionable
from synchros2.futures import wait_for_future
import synchros2.process as ros_process

def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("order", type=int)
    return parser

@ros_process.main(cli())
def main(args: argparse.Namespace) -> None:
    compute_fibonacci_sequence = Actionable(Fibonacci, "fibonacci")
    if not compute_fibonacci_sequence.wait_for_server(timeout_sec=5.0):
        print(f"No {compute_fibonacci_sequence.action_name} action server found")
        return
    print(f"Computing Fibonacci sequence for order N = {args.order}...")
    action = compute_fibonacci_sequence.asynchronously(
        Fibonacci.Goal(order=args.order), track_feedback=True
    )
    wait_for_future(action.acknowledgement, timeout_sec=5.0)
    if not action.acknowledged or not action.accepted:
        print("Computation rejected")
        return
    for feedback in action.feedback_stream(timeout_sec=5.0):
        print(f"Partial sequence is", feedback.sequence)
    if not wait_for_future(action.finalization, timeout_sec=5.0):
        print("Computation did not complete in time")
        action.cancel()
        return
    if action.succeeded:
        print("Sequence is", action.result.sequence)
    elif action.aborted:
        print("Computation aborted")
    elif action.cancelled:
        print("Computation cancelled")
    else:
        print("Internal server error")

if __name__ == "__main__":
    main()
```

**Note**: you may use servers in the `examples_rclpy_minimal_action_server` package to test this.

Action status must be checked explicitly, and timely before attempting to access an action's result or feedback (which may not be there yet).
Action acknowledgement and finalization futures can help synchronization. Action feedback streaming simplifies (soft) real-time action monitoring.

### Message feed APIs

These APIs wrap [`message_filters`](https://index.ros.org/r/message_filters) to provide a simpler UX when dealing with streams of messages, including but not limited to ROS 2 topic subscriptions.

Message feeds are the stateful generalization of standard ROS 2 message filters. Any message filter can become a feed, allowing:

- latest message retrieval
- message history retrieval
- message callback registration (and de-registration)
- message stream iteration as they arrive, one-by-one or in batches
- incoming message waits -- any message or those matching a predicate

Like message filters, most message feeds can be chained. This is true for all but those that externally source messages, ROS 2 topic subscriptions being the prime example of this. Other message feeds built into `synchros2` offer a vehicle for generic map-filter-reduce patterns, time synchronization across multiple message feeds, and synchronized `tf` lookups.

On word of caution. While any message filter can become a feed, standard ROS 2 message filters are usually not thread-safe. See [`synchros2.filters`](https://github.com/bdaiinstitute/ros_utilities/tree/main/synchros2/synchros2/filters.py) for thread-safe (re)implementations.

#### Common use cases

The following snippets make use of standard ROS 2 [`examples`](https://index.ros.org/r/examples/github-ros2-examples).
You may use the publishers in the `examples_rclpy_minimal_publisher` package to test these.

##### Looping over topic messages

```python
from contextlib import closing
from std_msgs.msg import String

from synchros2.subscription import Subscription
import synchros2.process as ros_process

@ros_process.main()
def main() -> None:
    topic_data = Subscription(String, "topic")
    with closing(topic_data.stream()) as stream:
        for message in stream:
            print(message.data)

if __name__ == "__main__":
    main()
```

Note that the topic message stream is managed and closed explicitly by the [`contextlib.closing`](https://docs.python.org/3/library/contextlib.html#contextlib.closing) context manager. This is important to stop message buffering as soon as it is no longer necessary. This can be deferred to the [garbage collector](https://docs.python.org/3/library/gc.html) but it may result in needless buffering for (much) longer.

##### Waiting for the next topic message

```python
from std_msgs.msg import String

from synchros2.subscription import Subscription
from synchros2.futures import unwrap_future
import synchros2.process as ros_process

@ros_process.main()
def main() -> None:
    topic_data = Subscription(String, "topic")
    while main.context.ok():
        message = unwrap_future(topic_data.update, timeout_sec=5.0)
        print(message.data)

if __name__ == "__main__":
    main()
```

Note that the future update is _unwrapped_ rather than waited on. If the future update does not become available in time, future unwrapping will raise.

##### Waiting for a specific topic message

```python
from std_msgs.msg import Int32
from std_msgs.msg import String

from synchros2.subscription import Subscription
from synchros2.futures import unwrap_future
import synchros2.process as ros_process

def to_int32(message: String) -> Int32:
    return Int32(data=int(message.data.rpartition(" ")[-1]))

@ros_process.main()
def main() -> None:
    topic_data = Subscription(String, "topic")
    while main.context.ok():
        message = unwrap_future(topic_data.matching_update(
            lambda message: to_int32(message).data % 5 == 0
        ), timeout_sec=5.0)
        print(message.data)

if __name__ == "__main__":
    main()
```

##### Setting up a message callback

```python
from std_msgs.msg import String

from synchros2.subscription import Subscription
import synchros2.process as ros_process

@ros_process.main()
def main() -> None:
    topic_data = Subscription(String, "topic")
    topic_data.recall(lambda message: print(message.data))
    main.wait_for_shutdown()

if __name__ == "__main__":
    main()
```

##### Synchronizing topic messages

```python
from contextlib import closing
from std_msgs.msg import String

from synchros2.feeds import SynchronizedMessageFeed
from synchros2.futures import unwrap_future
from synchros2.subscription import Subscription
import synchros2.process as ros_process

@ros_process.main()
def main() -> None:
    topics_data = SynchronizedMessageFeed(
        Subscription(String, "topic0"),
        Subscription(String, "topic1"),
        allow_headerless=True
    )
    with closing(topics_data.stream()) as stream:
        for a, b in stream:
            print("a:", a.data, "matches b:", b.data)

if __name__ == "__main__":
    main()
```

##### Adapting topic messages

```python
from contextlib import closing
from typing import Optional
from std_msgs.msg import String
from std_msgs.msg import Int32

from synchros2.feeds import AdaptedMessageFeed
from synchros2.subscription import Subscription
import synchros2.process as ros_process

def to_int32(message: String) -> Int32:
    return Int32(data=int(message.data.rpartition(" ")[-1]))

def keep_even(message: Int32) -> Optional[Int32]:
    return message if message.data % 2 == 0 else None

@ros_process.main()
def main() -> None:
    topic_data = AdaptedMessageFeed(
        Subscription(String, "topic"),
        lambda msg: keep_even(to_int32(msg))
    )
    with closing(topic_data.stream()) as stream:
        for message in stream:
            print(message.data)

if __name__ == "__main__":
    main()
```

Note that the adapter logic not only transforms the message type but also filters them. Returning `None` stops message propagation down the chain of message feeds.

##### Fetch the last 10 topic messages

```python
from contextlib import closing

from std_msgs.msg import String

from synchros2.subscription import Subscription
from synchros2.time import Duration
import synchros2.process as ros_process

@ros_process.main()
def main() -> None:
    clock = main.node.get_clock()
    with closing(Subscription(String, "topic", history_length=10)) as topic_data:
        print("Waiting for 5 seconds...")
        clock.sleep_until(clock.now() + Duration(seconds=5.0))
    for message in topic_data.history:
        print(message.data)

if __name__ == "__main__":
    main()
```

### Logging interoperability

To facilitate logging, `synchros2` include functionality to bridge code using Python standard [`logging`](https://docs.python.org/3/library/logging.html) with the ROS 2 logging system.

#### API review

##### Standard `logging`

```python
import logging

logging.basicConfig()
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
logger.info("Logged using standard logging")
```

These logs will be handled by Python's standard `logging` module (i.e. printed to console).

##### Standalone `rclpy` logging

```python
import rclpy.logging

logger = rclpy.logging.get_logger(__name__)
logger.set_level(rclpy.logging.LoggingSeverity.INFO)
logger.info("Logged using a standalone rclpy logger")
```

These logs will be ultimately handled by `rcutils` logging system (i.e. printed to console, perhaps forwarded to `spdlog`).

###### `rclpy` node logging

```python
import rclpy
import rclpy.logging

node = rclpy.create_node("my_node")
logger = node.get_logger()
logger.set_level(rclpy.logging.LoggingSeverity.INFO)
logger.info("Logged using an rclpy node logger")
```

These logs will be ultimately handled by `rcutils` logging system too, but will also be published to `/rosout`.

#### Log bridging

Managing multiple, independent logging systems is impractical. So is reworking a codebase to accommodate either. To avoid both scenarios, use `synchros2.logging.logs_to_ros` explicitly:

```python
import logging
import rclpy

from synchros2.logging import logs_to_ros

node = rclpy.create_node("my_node")
with logs_to_ros(node):
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.INFO)
    logger.info("Logged using standard logging")
```

or implicitly through [process-wide APIs](#process-wide-apis):

```bash
import logging

from synchros2.process as ros_process

@ros_process.main()
def main():
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.INFO)
    logger.info("Logged using standard logging")

if __name__ == "__main__":
    main()    
```

`logging` logs will propagate up the logger hierarchy and then forwarded to the logger of the corresponding node (the one provided in the first case, or the main prebaked node in the second case) and thus at least printed to console and published to `/rosout`. Note the above presumes that severity levels at every step of the way are such that the corresponding log will get through (in this example, INFO or below for both `logging` and ROS 2 logging system).

## Guidelines

### Integration testing

Unit testing ROS 2 code is no different from unit testing any other software, but some care must be exercised. 

#### Considerations for integration testing

* Data transport in ROS 2 is non-deterministic. So is callback execution when multi-threaded executors are in place. This is true for ROS 2 code in general, and for [process-wide APIs](#process-wide-apis) in particular, for which non-determinism is the price to pay for a synchronous programming model. As such, time sensitive and execution order dependent tests are bound to fail, even if only sporadically. Synchronization is necessary to avoid these issues, and fortunately the very same process-wide APIs enable safe use of synchronization primitives (e.g. via a multi-threaded executor spinning in a background thread, as provided by [`bdai_ros_wrappers.scope`](https://github.com/bdaiinstitute/ros_utilities/blob/main/synchros2/synchros2/scope.py) functionality).
* ROS 2 middlewares perform peer discovery by default. This allows distributed architectures in production but leads to cross-talk during parallelized testing. [`domain_coordinator`](https://github.com/ros2/ament_cmake_ros/tree/rolling/domain_coordinator) functionality simplifies [ROS domain ID](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Domain-ID.html) assignment enforcing host-wide uniqueness and with it, middleware isolation.

Therefore, as rules of thumb consider:

1. Using `synchros2.scope.top` to setup ROS 2 in your test fixtures.
   * Isolate it by passing a unique domain ID, as provided by `domain_coordinator.domain_id`.
1. Using synchronization primitives to wait with timeouts.
   * Note timeouts make the test time sensitive. Pick timeouts an order of magnitude above the expected test timing.

#### Writing integration tests using `pytest` (recommended)

[`pytest`](https://docs.pytest.org/en/7.4.x/) is a testing framework for Python software, the most common in ROS 2 Python codebases.

```python
import domain_coordinator
import pytest

from typing import Iterator

import synchros2.scope as ros_scope
from synchros2.action_client import ActionClientWrapper
from synchros2.futures import wait_for_future
from synchros2.scope import ROSAwareScope
from synchros2.single_goal_action_server import SingleGoalActionServer
from synchros2.subscription import wait_for_message

from std_msgs.msg import String
from std_srvs.srv import Trigger
from example_interfaces.action import Fibonacci

from rclpy.action.server import ServerGoalHandle
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy

@pytest.fixture
def ros() -> Iterator[ROSAwareScope]:
    """
    A pytest fixture that will set up and yield a ROS 2 aware global scope to each test that requests it.

    See https://docs.pytest.org/en/7.4.x/fixture.html for a primer on pytest fixtures.
    """
    with domain_coordinator.domain_id() as domain_id:  # to ensure node isolation 
        with ros_scope.top(global_=True, namespace="fixture", domain_id=domain_id) as top:
            yield top

def test_topic_pub_sub(ros: ROSAwareScope) -> None:
    """Asserts that a published message can be received on the other end."""
    qos_profile = QoSProfile(
        depth=100,
        history=HistoryPolicy.KEEP_LAST,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    pub = ros.node.create_publisher(String, "test", qos_profile)
    pub.publish(String(data="test"))
    # Message will arrive at an unspecified point in the future, thus
    assert wait_for_message(String, "test", qos_profile=qos_profile, timeout_sec=5.0)

def test_service_server_client(ros: ROSAwareScope) -> None:
    """Asserts that a service server replies to client requests."""
    def callback(_: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        response.success = True
        return response
    server = ros.node.create_service(Trigger, "trigger", callback)
    client = ros.node.create_client(Trigger, "trigger")
    assert client.wait_for_service(timeout_sec=5.0)
    future = client.call_async(Trigger.Request())
    assert wait_for_future(future, timeout_sec=5.0)
    response = future.result()
    assert response and response.success

def test_action_server_client(ros: ROSAwareScope) -> None:
    """Asserts that an action server reacts to action client requests."""
    def callback(goal_handle: ServerGoalHandle) -> Fibonacci.Result:
        result = Fibonacci.Result()
        result.sequence = [0, 1]
        for i in range(1, goal_handle.request.order):
            result.sequence.append(result.sequence[i] + result.sequence[i-1])
        goal_handle.succeed()
        return result
    server = SingleGoalActionServer(ros.node, Fibonacci, "compute", callback)
    client = ActionClientWrapper(Fibonacci, "compute", ros.node)
    assert client.wait_for_server(timeout_sec=5.0)
    goal = Fibonacci.Goal(order=3)
    result = client.send_goal_and_wait("compute", goal, timeout_sec=5.0)
    assert result and list(result.sequence) == [0, 1, 1, 2]
```

#### Writing integration tests using `unittest`

[`unittest`](https://docs.python.org/3/library/unittest.html) is the testing framework in Python's standard library.

```python
import contextlib
import domain_coordinator
import unittest

import synchros2.scope as ros_scope
from synchros2.action_client import ActionClientWrapper
from synchros2.futures import wait_for_future
from synchros2.scope import ROSAwareScope
from synchros2.single_goal_action_server import SingleGoalActionServer
from synchros2.subscription import wait_for_message

from std_msgs.msg import String
from std_srvs.srv import Trigger
from example_interfaces.action import Fibonacci

from rclpy.action.server import ServerGoalHandle
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy

class TestCase(unittest.TestCase):

    def setUp(self) -> None:
        """Sets up an isolated ROS 2 aware scope for all tests in the test case."""
        self.fixture = contextlib.ExitStack()
        domain_id = self.fixture.enter_context(domain_coordinator.domain_id())
        self.ros = self.fixture.enter_context(ros_scope.top(
            global_=True, namespace="fixture", domain_id=domain_id
        ))

    def tearDown(self) -> None:
        self.fixture.close()  # exits all contexts

    def test_topic_pub_sub(self) -> None:
        """Asserts that a published message can be received on the other end."""
        qos_profile = QoSProfile(
            depth=100,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        pub = self.ros.node.create_publisher(String, "test", qos_profile)
        pub.publish(String(data="test"))
        # Message will arrive at an unspecified point in the future, thus
        self.assertIsNotNone(wait_for_message(
            String, "test", qos_profile=qos_profile, timeout_sec=5.0))

    def test_service_server_client(self) -> None:
        """Asserts that a service server replies to client requests."""
        def callback(_: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
            response.success = True
            return response
        server = self.ros.node.create_service(Trigger, "trigger", callback)
        client = self.ros.node.create_client(Trigger, "trigger")
        self.assertTrue(client.wait_for_service(timeout_sec=5.0))
        future = client.call_async(Trigger.Request())
        self.assertTrue(wait_for_future(future, timeout_sec=5.0))
        response = future.result()
        self.assertIsNotNone(response)
        self.assertTrue(response.success)

    def test_action_server_client(self) -> None:
        """Asserts that an action server reacts to action client requests."""
        def callback(goal_handle: ServerGoalHandle) -> Fibonacci.Result:
            result = Fibonacci.Result()
            result.sequence = [0, 1]
            for i in range(1, goal_handle.request.order):
                result.sequence.append(result.sequence[i] + result.sequence[i-1])
            goal_handle.succeed()
            return result
        server = SingleGoalActionServer(self.ros.node, Fibonacci, "compute", callback)
        client = ActionClientWrapper(Fibonacci, "compute", self.ros.node)
        self.assertTrue(client.wait_for_server(timeout_sec=5.0))
        goal = Fibonacci.Goal(order=3)
        result = client.send_goal_and_wait("compute", goal, timeout_sec=5.0)
        self.assertIsNotNone(result)
        self.assertEqual(list(result.sequence), [0, 1, 1, 2])
```

#### Adding integration tests to a package

A package's type and build system dictate how unit tests are to be added. Unit tests for ROS 2 packages are typically hosted under the `test` subdirectory, so the following assumes this convention is observed. 

For `ament_cmake` packages, the `CMakeLists.txt` file should have:
```cmake
if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    # Define an arbitrary target for your tests such as:
    ament_add_pytest_test(unit_tests test)
endif()
```

For `ament_python` packages, the `setup.py` file should have:
```python
setup(
    # ...
    tests_require=['pytest'],
)
```

Note that `pytest` is the testing tool of choice regardless of package type.

**Note**: [test discovery mechanisms in `pytest`](https://docs.pytest.org/en/7.4.x/explanation/goodpractices.html#conventions-for-python-test-discovery) do not require `__init__.py` files under `test` directories.

#### Useful references

- [Official ROS 2 testing tutorial](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Testing/Testing-Main.html)
- [`pytest` documentation](https://pytest.org)
- [`unittest` documentation](https://docs.python.org/3/library/unittest.html)
