# Actions with synchros2

# Prerequisites

Make sure you’ve done the [Getting Started Guide](./GettingStartedGuide.md).

# Introduction

We’re going to create an action that just sleeps (as with the other tutorials sleeping is a placeholder for actual interesting work) and show how we can call it synchronously and asynchronously using the `Actionable` class.

# Writing and Running the Code

In this section we’ll write and run action code.  In the next section, we’ll analyze it.

## Action Interface

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
    
3. Add the action to `CMakeLists.txt` just above `msg/String.msg`: 
    
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
    

## Action Server

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
        if rclpy.ok() and goal_handle.is_active:
            goal_handle.succeed()
        return result

@ros_process.main()
def main() -> None:
    _ = WaitServer()
    ros_process.wait_for_shutdown()

if __name__ == "__main__":
    main()

```

## Action Client

The action client calls the action server.  We make both synchronous and asynchronous calls and show how to use both.  Create the file `<workspace>/src/synchros2_tutorials/synchros2_tutorials/wait_client.py` and add the following code: 

```python
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
        self._finished_printing_result = False
        self._client = Actionable(Wait, "wait_action")

    @property
    def finished_printing_result(self) -> bool:
        return self._finished_printing_result

    def initialize(self) -> bool:
        if not self._client.wait_for_server(timeout_sec=5.0):
            self._node.get_logger().error("Unable to find server for wait_action")
            return False
        return True

    def request_wait_synchronously(self, n_seconds_to_wait: float) -> Wait.Result | None:
        request = Wait.Goal()
        request.n_seconds_to_wait = n_seconds_to_wait
        return self._client.synchronous(request, feedback_callback=self._print_feedback, nothrow=True)

    def request_wait_asynchronously(self, n_seconds_to_wait: float) -> ActionFuture[Wait.Result, Wait.Feedback]:
        request = Wait.Goal()
        request.n_seconds_to_wait = n_seconds_to_wait
        future = self._client.asynchronous(request, track_feedback=True)
        self._finished_printing_result = False
        # Print when we're done
        future.finalization.add_done_callback(lambda x: self._print_future_result(future))
        return future

    def _print_feedback(self, feedback: Wait_FeedbackMessage) -> None:
        self._node.get_logger().info(f"There are {feedback.feedback.n_seconds_remaining} seconds remaining")

    def _print_future_result(self, action_future: ActionFuture[Wait.Result, Wait.Feedback]) -> None:
        try:
            result = action_future.result
        except RuntimeError as e:
            self._node.get_logger().error(f"Unable to get a result from the action.  Error was {e}")
            result = None
            return
        self._node.get_logger().info(f"Result after waiting asynchronously was {result}")
        self._finished_printing_result = True

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

    # Just wait for everything to finish
    node.get_logger().info("Waiting for action to finish")
    rate = node.create_rate(100)
    while rclpy.ok() and not client.finished_printing_result:
        rate.sleep()
    node.get_logger().info("All done goodbye")

if __name__ == "__main__":
    main()

```

## Running

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
    
    You should see over the next approximately 10 seconds output like: 
    
    ```bash
    [INFO] [1760021042.139412829] [wait_client]: Synchronous wait for 5s
    [INFO] [1760021042.346445216] [wait_client]: There are 4.899226188659668 seconds remaining
    [INFO] [1760021042.447699640] [wait_client]: There are 4.788418769836426 seconds remaining
    [INFO] [1760021042.448952209] [wait_client]: There are 4.778567314147949 seconds remaining
    <Output removed for brevity - it continues to count down>
    [INFO] [1760021047.126620118] [wait_client]: There are 0.019339799880981445 seconds remaining
    [INFO] [1760021047.136392909] [wait_client]: There are 0.009363651275634766 seconds remaining
    [INFO] [1760021047.147727235] [wait_client]: Result was synchros2_tutorials_interfaces.action.Wait_Result(n_seconds_waited=5.000729560852051)
    [INFO] [1760021047.147777862] [wait_client]: Asynchronous wait for 5s
    [INFO] [1760021047.147934286] [wait_client]: Finished requesting aysnc wait - making sure the action is acknowledged
    [INFO] [1760021047.151098932] [wait_client]: Printing feedback for 1s
    [INFO] [1760021047.172577991] [wait_client]: There are 4.999897003173828 seconds remaining
    [INFO] [1760021047.172679510] [wait_client]: There are 4.9891157150268555 seconds remaining
    <Output removed for brevity - it continues to count down>
    [INFO] [1760021048.143583256] [wait_client]: There are 4.009498596191406 seconds remaining
    [INFO] [1760021048.153375078] [wait_client]: There are 3.999499797821045 seconds remaining
    [INFO] [1760021048.153465333] [wait_client]: Stopping printing feedback after 1s
    [INFO] [1760021048.153553549] [wait_client]: Waiting for action to finish
    [INFO] [1760021052.156161341] [wait_client]: Result after waiting asynchronously was synchros2_tutorials_interfaces.action.Wait_Result(n_seconds_waited=5.000521183013916)
    [INFO] [1760021052.164679443] [wait_client]: All done goodbye
    
    ```
    

# Analyzing the Code

## Server

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

If we’re in the process of shutting down or the goal was aborted (no longer active), it’s already had a status set so we don’t want to do anything.  Otherwise we should succeed:

```python
        if rclpy.ok() and goal_handle.is_active:
            goal_handle.succeed()
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

## Client

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

We create our client class and store the ros scope node as per usual.  We also create a class variable (before any callbacks that might use that variable):

```python
class WaitClient:
    def __init__(self):
        self._node = ros_scope.ensure_node()
        self._finished_printing_result = False
```

We then create an `Actionable` which is a wrapper around `ActionClient` that supports `synchronous(ly)` and `asynchronous(ly)` calls (we refer to this colloquially often as a `synchros2` `Callable`).  Note that we don’t pass a node here - internally `Actionable` will use the ros scope node:

```python
        self._client = Actionable(Wait, "wait_action")
```

Before we can send goals to the action server, it needs to exist.  We wait for the action server in the `initialize` function with a 5 second timeout:

```python
    def initialize(self) -> bool:
        if not self._client.wait_for_server(timeout_sec=5.0):
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
        return self._client.synchronous(request, feedback_callback=self._print_feedback, nothrow=True)
```

This uses the `synchronous` call to the `Actionable` which will block until the call completes.  We pass two further arguments besides the action goal:

1. `feedback_callback`: This is a function that will receive the feedback and can process it.  Note that `synchros2` means that the feedback callback will *still* be called even though the call to the action server is blocking.
2. `nothrow`: By default `Actionable` will throw exceptions if the action does not complete successfully *even if it returns a result*.  This can be annoying when you want to use the result to find out how the action failed.  We recommend always using `nothrow` for this reason though it may mean the result can be `None`.

The client also offers a function that will request the wait from the action server and return immediately but also print the result and set `_finished_printing_result` when the action does return.  This is a little more complicated:

```python
    def request_wait_asynchronously(self, n_seconds_to_wait: float) -> ActionFuture[Wait.Result, Wait.Feedback]:
        request = Wait.Goal()
        request.n_seconds_to_wait = n_seconds_to_wait
        future = self._client.asynchronous(request, track_feedback=True)
```

We call the client’s `asynchronous` function which will send the goal and return immediately.  We also set `track_feedback` to `True` which means we’ll be able to access feedback from the future the call returns.  If your action doesn’t return feedback or you don’t care about the feedback, you can leave this as `False`.

The `asynchronous` call returns an [`ActionFuture`](../synchros2/action.py#L64) which gives us a lot of access to do things with the running action.  We take advantage of this right away in the function to set up a callback that will print when the action is done:

```python
        future.finalization.add_done_callback(lambda x: self._print_future_result(future))
```

`ActionFuture`s have two sub-futures: `acknowledgement` (which we’ll use in `main`) and `finalization`, corresponding to the two phases of an action:

1. `acknowledgement`: This is a future corresponding to whether the goal has been *acknowledged* essentially meaning that the server is running and not in some weird looped state (you can’t do anything to a `SingleGoalActionServer` that will fail to acknowledge an action).
2. `finalization`: This is a future corresponding to whether the action has finished in any way.  This will register as done if the goal isn’t acknowledged, the goal aborts, the goal cancels, the goal succeeds, etc.  Normally this is the future with which you want to interact.

In our case we want to run a function when the action has completely finished.  This corresponds to the `finalization` future so we add a done callback to it: `future.finalization.add_done_callback`.  The `add_done_callback` is a function common to all futures.  It takes a function that takes a single argument and returns `None`.  When the future is done, that function will be called and it will be passed the future.  Here’s a simpler example than the one above:

```python
def future_callback(future: Future) -> None:
    print(f"{future} is done!")

future = <some call that returns a future>    
future.add_done_callback(future_callback)
```

This will print when the future is done.

However, in our case we don’t want the `future.finalization` future.  That doesn’t have any of the information about the action that we need.  We want to pass the entire `future` to `_print_future_result` .  We could do this as follows

```python
        def _print_this_future(_) -> None:
            self._print_future_result(future)
        future.finalization.add_done_callback(_print_this_future)
```

We’ve defined an inline function that takes one argument (written as `_` because we never use it) and returns `None`.  Since this is exactly what `add_done_callback` needs, we can pass it to `add_done_callback`.  However, this is a somewhat clunky way of writing it (and doesn’t match the common ROS and ROS2 patterns you’ll see) so instead we use the Python `lambda` keyword to create an [anonymous function](https://www.w3schools.com/python/python_lambda.asp) - a function with no name.  The `lambda` keyword is written as `lambda arguments: expression` and creates a function that returns `expression(arguments)` when that function is called.  We’ve written `lambda x: self._print_future_result(future)`.  This is an anonymous function that takes one argument (`x`) and calls `self._print_future_result(future)` .  This is exactly the same as `_print_this_future` but we wrote it a lot more concisely.  Note that we are *not* at this moment in the code calling `self._print_future_result`.  We are just creating a function that will call that function when `future.finalization` is done.

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
            result = action_future.result
        except RuntimeError as e:
            self._node.get_logger().error(f"Unable to get a result from the action.  Error was {e}")
            result = None
        self._node.get_logger().info(f"Result after waiting asynchronously was {result}")
        self._finished_printing_result = True

```

If the action has failed to return a result calling `action_future.result` will cause a `RuntimeError`, which we check for and handle.  We also set `self._finished_printing_result` to `True` so we can check that we’ve finished processing the result of the action.

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

This returns an `ActionFuture` which we can use to interact with the action.  We first wait to make sure the goal is *acknowledged*.  This should be very fast - we’re essentially just checking that the server still exists.  Moreover, we won’t get any feedback until the action is acknowledged:

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

Finally we wait for the class to finish processing the result.  Note the use of `Rate` for the sleep to respect ROS time:

```python
    # Just wait for everything to finish
    node.get_logger().info("Waiting for action to finish")
    rate = node.create_rate(100)
    while rclpy.ok() and not client.finished_printing_result:
        rate.sleep()
    node.get_logger().info("All done goodbye")
```
