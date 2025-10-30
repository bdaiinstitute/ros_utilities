# Actionable and Serviced APIs

These APIs wrap those in `rclpy.client` and `rclpy.action.client` to provide a simpler UX for ROS 2 actions and services. They abstract calls behind an interface resembling remote procedure calls, supporting both synchronous and asynchronous invocation. Action futures build on the notion of a future to track actions' feedback, status, and result.

Both abstractions are well integrated with [ROS 2 aware scopes and processes](process_wide_apis.md).

## Services

Services in ROS 2 are for synchronous, request-response interactions -- ideal for quick queries or configuration changes. The client sends a request, the server replies immediately.

### Invoking a service synchronously

Serviced APIs allow you to invoke a service as you would with any other callable. Calls are synchronous by default, and all outcomes other than nominal success are signaled using exceptions. An optional timeout prevents blocking indefinitely.

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

### Invoking a service asynchronously

You can get a future service response instead of blocking on the call. Service response must be waited on, either explicitly and with a timeout or implicitly by early result request. Note fetching the future call result may raise.

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

## Actions

Actions in ROS 2 are for asynchronous, long-running tasks. The client sends a goal, the server executes and streams feedback, and the client can cancel if needed. Actions are suited for operations that may take time or require preemption.

### Invoking an action synchronously

Similarly, actionable APIs allow you to invoke an action as a callable. Calls are synchronous by default, and all outcomes other than nominal success are signaled using exceptions. Action feedback is ignored unless a callback is specified. An optional timeout prevents blocking indefinitely.

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

### Invoking an action asynchronously

You can get a future to an ongoing action instead of blocking on it. Action status must be checked explicitly, and timely before attempting to access an action's result or feedback (which may not be there yet). Action acknowledgement and finalization futures can help synchronization. Action feedback streaming simplifies online action monitoring.

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
