# Actions with `synchros2`

## Prerequisites

Make sure you’ve gone through the [getting started steps](../getting_started/index.md).

## Introduction

We’re going to create an action that just sleeps (as with the other tutorials sleeping is a placeholder for actual interesting work) and show how we can call it synchronously and asynchronously using the `Actionable` class.

## Writing and Running the Code

In this section we’ll write and run action code.  In the next section, we’ll analyze it.

### Action Interface

As with any ROS action, we need to start by defining the action message:

1. Create an `action` directory in `synchros2_tutorials_interfaces`: 
    
    ```python
    mkdir <workspace>/src/synchros2_tutorials_interfaces/action
    ```
    
2. Create the file `<workspace>/src/synchros2_tutorials_interfaces/action/Wait.action` and add the following text: 
    
    ```python
    # How long to wait
    float32 n_seconds_to_wait
    ---
    # How long we actually waited
    float32 n_seconds_waited
    ---
    # How long is left
    float32 n_seconds_remaining
    ```
    
3. Add the action to `CMakeLists.txt` just above `msg/String.msg`.  When you've done this the full `rosidl_generate_interfaces` block should look as follows (don't create a new block!):
    
    ```python
    rosidl_generate_interfaces(${PROJECT_NAME}
      "action/Wait.action"
      "msg/String.msg"
    )
    
    ```
    
4. Build and source the interface package: 
    
    ```bash
    cd <workspace>
    colcon build --packages-select synchros2_tutorials_interfaces
    source install/setup.bash
    ```
    
5. Check that we see the new action: 
    
    ```bash
    ros2 interface show synchros2_tutorials_interfaces/action/Wait
    ```
    
    should print: 
    
    ```bash
    # How long to wait
    float32 n_seconds_to_wait
    ---
    # How long we actually waited
    float32 n_seconds_waited
    ---
    # How long is left
    float32 n_seconds_remaining
    ```
    If `ros2 interface` cannot find the package make sure you:
    * Sourced `install/setup.bash` in your current terminal *after* building.
    * Built from `<workspace>` and not any other directory.


### Action Server

The action server will wait for the requested time.  Create the file `<workspace>/src/synchros2_tutorials/synchros2_tutorials/wait_server.py` and add the following code: 

```python
import rclpy
from rclpy.action.server import ServerGoalHandle
from synchros2_tutorials_interfaces.action import Wait

import synchros2.process as ros_process
import synchros2.scope as ros_scope
from synchros2.single_goal_action_server import SingleGoalActionServer

class WaitServer:
    def __init__(self):
        self._node = ros_scope.ensure_node()
        self._server = SingleGoalActionServer(self._node, Wait, "wait_action", self._goal_callback)
        self._node.get_logger().info("Ready to serve wait requests!")

    def _time_in_seconds(self):
        curr_time = self._node.get_clock().now()
        return float(curr_time.nanoseconds) / 1e9

    def _goal_callback(self, goal_handle: ServerGoalHandle) -> Wait.Result:
        start_time = self._time_in_seconds()
        n_seconds_to_wait = goal_handle.request.n_seconds_to_wait
        rate = self._node.create_rate(100)  # Check 100 times per second whether we're done
        while (
            rclpy.ok()
            and goal_handle.is_active
            and not goal_handle.is_cancel_requested
            and self._time_in_seconds() - start_time < n_seconds_to_wait
        ):
            feedback = Wait.Feedback()
            feedback.n_seconds_remaining = n_seconds_to_wait - (self._time_in_seconds() - start_time)
            goal_handle.publish_feedback(feedback)
            rate.sleep()

        result = Wait.Result()
        result.n_seconds_waited = self._time_in_seconds() - start_time

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif rclpy.ok() and goal_handle.is_active:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

@ros_process.main()
def main() -> None:
    _ = WaitServer()
    ros_process.wait_for_shutdown()

if __name__ == "__main__":
    main()

```

### Action Client

The action client calls the action server.  We make both synchronous and asynchronous calls and show how to use both.  Create the file `<workspace>/src/synchros2_tutorials/synchros2_tutorials/wait_client.py` and add the following code: 

```python
import time

import rclpy
from synchros2_tutorials_interfaces.action import Wait

# The message type for feedback callbacks is weird
from synchros2_tutorials_interfaces.action._wait import Wait_FeedbackMessage

import synchros2.process as ros_process
import synchros2.scope as ros_scope
from synchros2.action import Actionable, ActionFuture
from synchros2.futures import wait_for_future

class WaitClient:
    def __init__(self):
        self._node = ros_scope.ensure_node()
        self._wait = Actionable(Wait, "wait_action")

    def initialize(self) -> bool:
        if not self._wait.wait_for_server(timeout_sec=5.0):
            self._node.get_logger().error("Unable to find server for wait_action")
            return False
        return True

    def request_wait_synchronously(self, n_seconds_to_wait: float) -> Wait.Result | None:
        request = Wait.Goal()
        request.n_seconds_to_wait = n_seconds_to_wait
        return self._wait.synchronously(request, feedback_callback=self._print_feedback, nothrow=True)

    def request_wait_asynchronously(self, n_seconds_to_wait: float) -> ActionFuture[Wait.Result, Wait.Feedback]:
        request = Wait.Goal()
        request.n_seconds_to_wait = n_seconds_to_wait
        return self._wait.asynchronously(request, track_feedback=True, done_callback=self._print_future_result)

    def _print_feedback(self, feedback: Wait_FeedbackMessage) -> None:
        self._node.get_logger().info(f"There are {feedback.feedback.n_seconds_remaining} seconds remaining")

    def _print_future_result(self, action_future: ActionFuture[Wait.Result, Wait.Feedback]) -> None:
        try:
            self._node.get_logger().info(f"Result after waiting asynchronously was {action_future.result}")
            self._node.get_logger().info("Sleeping for one second")
            time.sleep(1.0)
        except RuntimeError as e:
            self._node.get_logger().error(f"Unable to get a result from the action.  Error was {e}")


@ros_process.main()
def main() -> None:
    node = ros_scope.ensure_node()
    client = WaitClient()
    if not client.initialize():
        return
    # Send a 5s wait and just wait here
    node.get_logger().info("Synchronous wait for 5s")
    result = client.request_wait_synchronously(5.0)
    node.get_logger().info(f"Result was {result}")

    # Send a 5s wait asynchronously
    node.get_logger().info("Asynchronous wait for 5s")
    action_future = client.request_wait_asynchronously(5.0)

    # Acknowledgement means checking the server is running - this should be fast
    node.get_logger().info("Finished requesting aysnc wait - making sure the action is acknowledged")
    wait_for_future(action_future.acknowledgement, timeout_sec=5.0)
    if not action_future.acknowledged or not action_future.accepted:
        # Something bad happened
        node.get_logger().error("Action was rejected!")
        return

    # Print feedback for a while
    node.get_logger().info("Printing feedback for 1s")
    start_time = node.get_clock().now()
    for feedback in action_future.feedback_stream():
        node.get_logger().info(f"There are {feedback.n_seconds_remaining} seconds remaining")
        if (node.get_clock().now() - start_time).nanoseconds / 1e9 >= 1.0:
            node.get_logger().info("Stopping printing feedback after 1s")
            break

    # Wait for the action to finish
    node.get_logger().info("Waiting for action to finish")
    wait_for_future(action_future.finalization, timeout_sec=20.0)
    node.get_logger().info("All done goodbye")

if __name__ == "__main__":
    main()

```

### Running

We need to run both the action server and client.  The client will wait for the action server to start for up to 5 seconds, but it’s better to start the server first so we don’t have to worry about that.

1. In one terminal (make sure you’ve sourced `<workspace>/install/setup.bash`), start the action server: 
    
    ```bash
    cd <workspace>/src/synchros2_tutorials/synchros2_tutorials
    python wait_server.py
    ```
    
    This should print `Ready to serve wait requests!` .
    
2. In a second terminal (make sure you’ve sourced `<workspace>/install/setup.bash`), start the action client: 
    
    ```bash
    cd <workspace>/src/synchros2_tutorials/synchros2_tutorials
    python wait_client.py
    ```
    
    You should see over the next approximately 11 seconds output like: 
    
    ```bash
    [INFO] [1761586276.653590123] [wait_client]: Synchronous wait for 5s
    [INFO] [1761586276.858891226] [wait_client]: There are 4.8993120193481445 seconds remaining
    [INFO] [1761586276.960031113] [wait_client]: There are 4.789267539978027 seconds remaining
    <output skipped>
    [INFO] [1761586281.639260495] [wait_client]: There are 0.0193941593170166 seconds remaining
    [INFO] [1761586281.649156489] [wait_client]: There are 0.009440422058105469 seconds remaining
    [INFO] [1761586281.660667920] [wait_client]: Result was synchros2_tutorials_interfaces_example.action.Wait_Result(n_seconds_waited=5.000578880310059)
    [INFO] [1761586281.660778468] [wait_client]: Asynchronous wait for 5s
    [INFO] [1761586281.661065689] [wait_client]: Finished requesting aysnc wait - making sure the action is acknowledged
    [INFO] [1761586281.664718017] [wait_client]: Printing feedback for 1s
    [INFO] [1761586281.687815907] [wait_client]: There are 4.999876976013184 seconds remaining
    [INFO] [1761586281.687980344] [wait_client]: There are 4.98870325088501 seconds remaining
    <output skipped>
    [INFO] [1761586282.655594633] [wait_client]: There are 4.009243488311768 seconds remaining
    [INFO] [1761586282.665671291] [wait_client]: There are 3.9990577697753906 seconds remaining
    [INFO] [1761586282.665801514] [wait_client]: Stopping printing feedback after 1s
    [INFO] [1761586282.665864634] [wait_client]: Waiting for action to finish
    [INFO] [1761586286.667215310] [wait_client]: Result after waiting asynchronously was synchros2_tutorials_interfaces_example.action.Wait_Result(n_seconds_waited=5.001377582550049)
    [INFO] [1761586286.667313122] [wait_client]: Sleeping for one second
    [INFO] [1761586287.667953144] [wait_client]: All done goodbye
    ```

## Analyzing the Code

### Server

The server should receive a goal that tells it to wait for a certain amount of time and then publish feedback about how long it has left to wait.  We will analyze the code to see how it does this.

The first imports are the imports we need for most ROS2 action servers: `rclpy`, `ServerGoalHandle` (we’ll talk more about how that’s used), and our message definition `Wait`:

```python
import rclpy
from rclpy.action.server import ServerGoalHandle
from synchros2_tutorials_interfaces.action import Wait
```

We then have the imports we need for any `synchros2` process (non-main files don’t need the `ros_process` import):

```python
import synchros2.process as ros_process
import synchros2.scope as ros_scope
```

Finally we import the `SingleGoalActionServer`:

```python
from synchros2.single_goal_action_server import SingleGoalActionServer
```

This is a wrapper around the ROS2 action server that assumes that we only want to serve a single goal at a time (similar to `SimpleActionServer` from ROS1).  When a new goal comes in, the `SingleGoalActionServer` will automatically send a cancel request to the currently running goal.  We recommend using this server for any simple actions.

On initialization, the `WaitServer` saves the ros scope node as we normally do:

```python
class WaitServer:
    def __init__(self):
        self._node = ros_scope.ensure_node()
```

It then creates the goal server:

```python
        self._server = SingleGoalActionServer(self._node, Wait, "wait_action", self._goal_callback)
```

This sets up an action server on the `wait_action` topic.  When a goal is received, it will be sent to `self._goal_callback`.  Note that we, as always, use the ros scope node for the action server.

Because we do a lot of waiting we add a helper function that gives us the current time in seconds rather than an `rclpy.time.Time` object:

```python
    def _time_in_seconds(self):
        curr_time = self._node.get_clock().now()
        return float(curr_time.nanoseconds) / 1e9
```

Now we have our goal callback.  Like all ROS2 goal callbacks it takes a `ServerGoalHandle`, which allows us to interact with the action server:

```python
    def _goal_callback(self, goal_handle: ServerGoalHandle) -> Wait.Result:
        start_time = self._time_in_seconds()
```

For example the first thing we do is access the goal that was sent and find out how many seconds to wait.  The action goal will always be accessible with the `request` field.

```python
        n_seconds_to_wait = goal_handle.request.n_seconds_to_wait
```

We want to publish feedback so instead of just sleeping for `n_seconds_to_wait` we use a `while` loop (we could make this more precise if needed but that just adds extra complication to this example).  We use a rate to let us sleep for 0.01s during the loop in a way that respects ROS time (important for playback or simulation):

```python
        rate = self._node.create_rate(100)  # Check 100 times per second whether we're done

```

Now we enter the while loop:

```python
        while (
            rclpy.ok()
            and goal_handle.is_active
            and not goal_handle.is_cancel_requested
            and self._time_in_seconds() - start_time < n_seconds_to_wait
        ):
```

This loop has 4 conditions:

1. `rclpy.ok()`: This checks that there hasn’t been a `Ctrl+C` .  If you don’t add this check, the action server won’t exit on `Ctrl+C` until it’s done with the loop.
2. `goal_handle.is_active`: This checks that nothing has aborted the goal outside of the server.  There isn’t really a way in this server that the goal handle can become inactive but it’s a good check to be in the habit of including.
3. `not goal_handle.is_cancel_requested`: This checks that we aren’t trying to cancel the current goal.  **This is a very important check!**  The `SingleGoalActionServer` automatically sends a cancel request that will show up as `goal_handle.is_cancel_requested = True` but it doesn’t have the power to actually stop the execution of the current goal.  That’s up to the user!  If we see a cancel, we need to stop our server and return ASAP.
4. `self._time_in_seconds() - start_time < n_seconds_to_wait`: This is waiting for the requested length of time

In the loop we publish our feedback using the goal handle:

```python
            feedback = Wait.Feedback()
            feedback.n_seconds_remaining = n_seconds_to_wait - (self._time_in_seconds() - start_time)
            goal_handle.publish_feedback(feedback)
```

and then we sleep for 0.01s (ROS time) using our `Rate`:

```python
            rate.sleep()
```

Once we’re out of the loop, we’re done executing and can fill in the result:

```python
        result = Wait.Result()
        result.n_seconds_waited = self._time_in_seconds() - start_time
```

We also need to set a status on the goal and there are three ways to exit the while loop in a way that could have made the goal unsuccessful.  If the goal had a cancel requested, we want to cancel it:

```python
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
```

If we’re not in the process of shutting down and our goal hand is still active, we should succeed:

```python
        elif rclpy.ok() and goal_handle.is_active:
            goal_handle.succeed()
```

Otherwise something has gone wrong and we should abort:
```python
	else:
	    goal_handle.abort()
```


Finally we return the result, which the ROS2 mechanisms will then send to our client:

```python
        return result
```

We always decorate our main function with `@ros_process.main()`!  This allows us to block in callbacks if we need to (note that we created our action server using the ros scope node):

```python
@ros_process.main()
def main() -> None:
```

Inside the main function we create our server and then just wait for shutdown so that it remains alive and listening:

```python
    _ = WaitServer()
    ros_process.wait_for_shutdown()
```

Note that `_` is just Python syntax for “we won’t reference this variable again”.

### Client

The client sends goals to the server.  We show how to do this both synchronously and asynchronously.

We start by importing `rclpy` and the message definition:

```python
import rclpy
from synchros2_tutorials_interfaces.action import Wait
```

We also import the message type that ROS2 passes to feedback callbacks.  This isn’t `Wait.Feedback` but a wrapped feedback type that only seems to be available for import from the private module.  We only need this for type hinting:

```python
# The message type for feedback callbacks is weird
from synchros2_tutorials_interfaces.action._wait import Wait_FeedbackMessage
```

We then have the imports we need for any `synchros2` process (non-main files don’t need the `ros_process` import):

```python
import synchros2.process as ros_process
import synchros2.scope as ros_scope
```

Finally we import the useful classes and functions from `synchros2` for an action client.  We’ll discuss these in more detail as we use them:

```python
from synchros2.action import Actionable, ActionFuture
from synchros2.futures import wait_for_future
```

We create our client class and store the ros scope node as per usual:

```python
class WaitClient:
    def __init__(self):
        self._node = ros_scope.ensure_node()
```

We then create an `Actionable` which is a wrapper around `ActionClient` that supports `synchronous(ly)` and `asynchronous(ly)` calls (we refer to this colloquially often as a `synchros2` `Callable`).  Note that we don’t pass a node here - internally `Actionable` will use the ros scope node:

```python
        self._wait = Actionable(Wait, "wait_action")
```

Before we can send goals to the action server, it needs to exist.  We wait for the action server in the `initialize` function with a 5 second timeout:

```python
    def initialize(self) -> bool:
        if not self._wait.wait_for_server(timeout_sec=5.0):
            self._node.get_logger().error("Unable to find server for wait_action")
            return False
        return True
```

We could wait for the server in `__init__` but `__init__` can’t return so we’d have to check separately if the wait was successful.  We use a 5 second timeout here - we don’t have to use a timeout but failing to do so can result in the program hanging in confusing ways when there are multiple action clients at play and it’s not clear which one is blocked.

The action client offers a function to request a wait from the action server and block until the result can be returned:

```python
    def request_wait_synchronously(self, n_seconds_to_wait: float) -> Wait.Result | None:
        request = Wait.Goal()
        request.n_seconds_to_wait = n_seconds_to_wait
        return self._wait.synchronously(request, feedback_callback=self._print_feedback, nothrow=True)
```

This uses the `synchronously` call to the `Actionable` which will block until the call completes.  We pass two further arguments besides the action goal:

1. `feedback_callback`: This is a function that will receive the feedback and can process it.  Note that `synchros2` means that the feedback callback will *still* be called even though the call to the action server is blocking.
2. `nothrow`: By default `Actionable` will throw exceptions if the action does not complete successfully *even if it returns a result*.  If you don't care to handle the various ways the action can fail without returning a result (e.g. action server doesn't exist, etc) then `nothrow` is a simple way of getting a result even when the result indicates a failure.  We use it here for simplicity but it is better practice to catch and handle all of the possible exceptions - see the [Action Exceptions section](#Action-Exceptions) for more details.

The client also offers a function that will request the wait from the action server and return immediately but also print the result when the action does return:

```python
    def request_wait_asynchronously(self, n_seconds_to_wait: float) -> ActionFuture[Wait.Result, Wait.Feedback]:
        request = Wait.Goal()
        request.n_seconds_to_wait = n_seconds_to_wait
        return self._wait.asynchronously(request, track_feedback=True, done_callback=self._print_future_result)
```

We call the client’s `asynchronously` function which will send the goal and return immediately.  We also set `track_feedback` to `True` which means we’ll be able to access feedback from the future the call returns.  If your action doesn’t return feedback or you don’t care about the feedback, you can leave this as `False`.  We pass the `_print_future_result` as the `done_callback`so that we'll print the result when finished.  We could also pass `_print_feedback` as the `feedback_callback` but for illustration purposes, we do the printing in `main` instead.

The `asynchronously` call returns an [`ActionFuture`](../synchros2/action.py#L64) which gives us a lot of access to do things with the running action, which we take advantage of in `main`.

We then have a couple utility functions we use for printing:

```python
    def _print_feedback(self, feedback: Wait_FeedbackMessage) -> None:
        self._node.get_logger().info(f"There are {feedback.feedback.n_seconds_remaining} seconds remaining")
```

Feedback callbacks don’t take a `Feedback` but rather a `FeedbackMessage`.  The `FeedbackMessage` has a `goal_id` field and a `feedback` field that contains the actual `Feedback`.  We access that field to print the feedback.

Finally we have the function we use to print the result:

```python
    def _print_future_result(self, action_future: ActionFuture[Wait.Result, Wait.Feedback]) -> None:
        try:
            self._node.get_logger().info(f"Result after waiting asynchronously was {action_future.result}")
            self._node.get_logger().info("Sleeping for one second")
            time.sleep(1.0)
        except RuntimeError as e:
            self._node.get_logger().error(f"Unable to get a result from the action.  Error was {e}")
```

The function sleeps to show that the action will not fully finalize until this callback is done.  If the action has failed to return a result calling `action_future.result` will cause a `RuntimeError`, which we check for and handle.

Finally, we have our main function in which we use the various features of the client.  We begin, as always by decorating with `ros_process.main()`

```python
@ros_process.main()
def main() -> None:
```

We assign `node` to `ros_scope.ensure_node()` to make it shorter to type:

```python
    node = ros_scope.ensure_node()
```

and we create and initialize our client:

```python
    client = WaitClient()
    if not client.initialize():
        return
```

We first make a synchronous call, which works like any blocking function call:

```python
   # Send a 5s wait and just wait here
    node.get_logger().info("Synchronous wait for 5s")
    result = client.request_wait_synchronously(5.0)
    node.get_logger().info(f"Result was {result}")
```

During this time we see the feedback printed due to the fact we assigned `feedback_callback` (also showing we can block *and* have a working callback).

We then send an asynchronous goal.  This will return immediately:

```python
    # Send a 5s wait asynchronously
    node.get_logger().info("Asynchronous wait for 5s")
    action_future = client.request_wait_asynchronously(5.0)
```

This returns an `ActionFuture` which we can use to interact with the action.  `ActionFuture`s have two sub-futures: `acknowledgement` and `finalization`, corresponding to the two phases of an action:

1. `acknowledgement`: This is a future corresponding to whether the goal has been *acknowledged* essentially meaning that the server is running and not in some weird looped state (you can’t do anything to a `SingleGoalActionServer` that will fail to acknowledge an action but more complex action servers can reject goals if they are busy).
2. `finalization`: This is a future corresponding to whether the action has finished in any way.  This will register as done if the goal isn’t acknowledged, the goal aborts, the goal cancels, the goal succeeds, etc.


We first wait to make sure the goal is *acknowledged*.  This should be very fast - we’re essentially just checking that the server still exists.  Moreover, we won’t get any feedback until the action is acknowledged:

```python
    # Acknowledgement means checking the server is running - this should be fast
    node.get_logger().info("Finished requesting aysnc wait - making sure the action is acknowledged")
    wait_for_future(action_future.acknowledgement, timeout_sec=5.0)
    if not action_future.acknowledged or not action_future.accepted:
        # Something bad happened
        node.get_logger().error("Action was rejected!")
        return
```

In order to wait until the goal is acknowledged we use `wait_for_future` (from `synchros2` so that it won’t block callbacks etc) on the `action_future.acknowledgement` future.  This is the future that is tracking whether the goal is acknowledged.

Once a goal has been acknowledged, it will start to produce feedback.  One of the nice features of the `ActionFuture` is the `feedback_stream`, which will generate feedback messages and can be used in a for loop.  Note that this will only work if we called the action originally with `track_feedback=True` .  The feedback in `feedback_stream` is an actual `Feedback` not the `FeedbackMessage` wrapper that the feedback callback gets:

```python
    for feedback in action_future.feedback_stream():
        node.get_logger().info(f"There are {feedback.n_seconds_remaining} seconds remaining")
```

Because we wanted to show both feedback stream and how to wait for an asynchronous action, we only print the feedback for one second.  If we let the for loop keep going, it would exit when the action finishes.

Then we wait for the action to fully finish.  This is the `action.finalization` future.  This will take approximately 4 seconds for the server to return and then another second for the class callback to return:
```python
    # Wait for the action to finish
    node.get_logger().info("Waiting for action to finish")
    wait_for_future(action_future.finalization, timeout_sec=20.0)
    node.get_logger().info("All done goodbye")
```

## Action Exceptions

In the above example we use `nothrow` for simplicity because we know our action server is unlikely to experience a catastrophic failure.  However it is better practice to explicitly handle the possible exceptions:
* `ActionTimeout`: Action timed out before a result could be obtained.
* `ActionRejected`: The server rejected the action (possibly it didn't exist, possibly it was busy).
* `ActionCancelled`: Something external to the server cancelled the action before a result could be obtained.  This is often because a new goal came in and preempted the old goal.
* `ActionAborted`: The server finished but decided the action was aborted.  Often this is because something went wrong during action execution.  `ActionAborted` usually comes with a result that can explain what went wrong.
* `ActionException`: Something else went wrong not covered by the above.

So a better version of the `request_wait_synchronously` function is:

```python
from synchros2.action import (
    ActionAborted,
    ActionCancelled,
    ActionException,
    ActionRejected,
    ActionTimeout,
)

    ...

    def request_wait_synchronously(self, n_seconds_to_wait: float) -> Wait.Result | None:
        request = Wait.Goal()
        request.n_seconds_to_wait = n_seconds_to_wait
        try:
            return self._wait.synchronously(request, feedback_callback=self._print_feedback)
        except ActionAborted as e:
            # The action failed in some way that still produced a result so just return that.
            return e.action.result
        except ActionCancelled:
            self._node.get_logger().error("Wait action was cancelled")
            return None
        except ActionRejected:
            self._node.get_logger().error("Wait server did not accept the wait request")
            return None
        except ActionTimeout:
            self._node.get_logger().error("Wait action timed out")
            return None
        except ActionException as e:
            self._node.get_logger().error(f"Wait action encountered an exception: {e}")
            return None
```

## Next Steps

Now try the [TF tutorial](tf.md).
