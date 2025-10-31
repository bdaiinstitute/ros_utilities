# Using TF with synchros2

This tutorial will guide you through the basics of using the Transform (TF) system with the `synchros2` utilities package.

## Prerequisites

Make sure you’ve done the [Getting Started Guide](./GettingStartedGuide.md).

## What is TF?

[TF](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html) is a ROS library that lets you keep track of multiple coordinate frames over time, and helps transform data between them. For robots, this is essential for tasks like localization, navigation, and sensor fusion.

## What does `synchros2` do for TF?

TF essentially operates by publishing a bunch of messages on a topic and then having a buffer that listens to and stores these messages.  Because this needs to happen in the background and involves a lot of data, TF listeners tend to be one of the most sensitive parts of the system in native ROS2 to multi-threading, often leading to not receiving transforms or receiving them with high latency even when allowed to run in its own thread.  `synchros2` handles the background threading for a TF listener.

## Transform Listeners and `synchros2`

There are two ways to use transforms listeners and `synchros2`:

1. Set `uses_tf=True` in the main decorator:
  ```python
  import synchros2.process as ros_process
  import synchros2.scope as ros_scope

  @ros_process.main(uses_tf=True)
  def main():
      tf_listener = ros_scope.tf_listener()
  ```

2. Use the `TFListenerWrapper`:
  ```python
   from synchros2.tf_listener_wrapper import TFListenerWrapper

   # Create listener (node can be provided or will be created automatically)
   tf_listener = TFListenerWrapper()
   ```

We'll give examples of both of these.

### Transform Listener and `uses_tf`

#### Writing and Running the Code

1. Create the file `<workspace>/src/synchros2_tutorials/synchros_tutorials/uses_tf.py` and add the following code:

```python
import synchros2.process as ros_process
import synchros2.scope as ros_scope

@ros_process.main(uses_tf=True)
def main() -> None:
    node = ros_scope.node()
    # The process-wide TF listener is available everywhere as ros_scope.tf_listener()
    tf_listener = ros_scope.tf_listener()
    base_frame = "world"
    target_frame = "robot"
    # Wait for our transform to become available
    node.get_logger().info(f"Waiting for transform from {base_frame} to {target_frame} to be available")
    tf_listener.wait_for_a_tform_b(base_frame, target_frame)
    world_t_robot = tf_listener.lookup_a_tform_b(base_frame, target_frame)
    node.get_logger().info(f"Transform from world to robot is {world_t_robot}")

if __name__ == "__main__":
    main()
```

2. Run the code.  It should hang at "Waiting for transform from world to robot to be available:

```bash
cd <workspace>/src/synchros2_tutorials/synchros2_tutorials
python uses_tf.py
```

3. Publish a transform from `world` to `robot` for the listener to hear:
```bash
ros2 run tf2_ros static_transform_publisher --frame-id world --child-frame-id robot --x 1.0 --y -1.0 --z 2.0 --yaw 3.14
```

4. In the terminal running `uses_tf`, you should see output like:
```bash
[INFO] [1761941125.032417530] [uses_tf]: Waiting for transform from world to robot to be available
[INFO] [1761941125.145993152] [uses_tf]: Transform from world to robot is geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id='world'), child_frame_id='robot', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=1.0, y=-1.0, z=2.0), rotation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.9999996829318346, w=0.0007963267107332631)))
```

#### Analyzing the Code

We start with the imports all `synchros2` code needs (non-main code doesn’t need the `ros_process` one):

```python
import synchros2.process as ros_process
import synchros2.scope as ros_scope
```

As with all `synchros2` code, our main function is decorated with `ros_process.main`:

```python
@ros_process.main(uses_tf=True)
def main() -> None:
```
However, this time we've passed the argument `uses_tf=True`.  This causes `synchros2` to create a global TF listener that can be accessed from anywhere.  Since TF listeners essentially just sit and gather data, having a single one running in the background is both efficient and useful.  We can access this TF listener with:

```python
    # The process-wide TF listener is available everywhere as ros_scope.tf_listener
    tf_listener = ros_scope.tf_listener()
```

When you first create a TF listener, you must wait until it gets some messages before being able to use it (another reason it's helpful to have one that just starts when the process does).  So it's usually a good idea to wait for the transform to become available (see the next section for a way to do this as part of the lookup call) even if you expect this process to start after the transform is broadcast.  Because the TF listener is running in a background thread, we can block waiting for a transform and not block the message collection that the TF listener needs:

```python
    # Wait for our transform to become available
    node.get_logger().info(f"Waiting for transform from {base_frame} to {target_frame} to be available")
    tf_listener.wait_for_a_tform_b(base_frame, target_frame)
```

Once the transform is available we can look it up:

```python
    world_t_robot = tf_listener.lookup_a_tform_b(base_frame, target_frame)
    node.get_logger().info(f"Transform from world to robot is {world_t_robot}")
```

### Using `TransformListenerWrapper`

If you don't want to create a process-wide transform listener, you can still use the `synchros2` mechanisms to run a TF listener in the background by using `synchros2`'s `TFListenerWrapper` class.  We'll show in this section how to do that.

#### Writing and Running the Code

1. Create the file `<workspace>/src/synchros2_tutorials/synchros_tutorials/tf_wrapper_example.py` and add the following code:

```python
import synchros2.process as ros_process
import synchros2.scope as ros_scope
from synchros2.tf_listener_wrapper import TFListenerWrapper

@ros_process.main()
def main() -> None:
    tf_listener = TFListenerWrapper()
    node = ros_scope.node()
    base_frame = "world"
    target_frame = "robot"
    # Wait for our transform to become available
    node.get_logger().info(f"Looking up transform from {base_frame} to {target_frame}")
    world_t_robot = tf_listener.lookup_a_tform_b(base_frame, target_frame, wait_for_frames=True)
    node.get_logger().info(f"Transform from world to robot is {world_t_robot}")

if __name__ == "__main__":
    main()
```

2. Run the listener code.  It should hang at "Looking up transform":
```bash
cd <workspace>/src/synchros2_tutorials/synchros2_tutorials
python tf_wrapper_example.py
```

3. Publish a transform from `world` to `robot` for the listener to hear:
```bash
ros2 run tf2_ros static_transform_publisher --frame-id world --child-frame-id robot --x 1.0 --y -1.0 --z 2.0 --yaw 3.14
```

4. In the terminal running the listener code, you should see output like:
```bash
[INFO] [1761942904.226617208] [tf_wrapper_example]: Looking up transform from world to robot
[INFO] [1761942907.715904380] [tf_wrapper_example]: Transform from world to robot is geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id='world'), child_frame_id='robot', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=1.0, y=-1.0, z=2.0), rotation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.9999996829318346, w=0.0007963267107332631)))
```

#### Analyzing the Code

We start with the imports all `synchros2` code needs (non-main code doesn’t need the `ros_process` one):

```python
import synchros2.process as ros_process
import synchros2.scope as ros_scope
```

We also import the TF listener wrapper since we'll be creating one:

```python
from synchros2.tf_listener_wrapper import TFListenerWrapper
```

As with all `synchros2` code, our main function is decorated with `ros_process.main`.  This time we don't include `uses_tf` since we are going to create our own listener:

```python
@ros_process.main()
def main() -> None:
```

Now we create a TF listener wrapper.  This can be optionally passed a node but will use the ros scope node if we don't pass it one:

```python
    tf_listener = TFListenerWrapper()
```

Then we use the listener to look up the transform

```python
    # Wait for our transform to become available
    node.get_logger().info(f"Looking up transform from {base_frame} to {target_frame}")
    world_t_robot = tf_listener.lookup_a_tform_b(base_frame, target_frame, wait_for_frames=True)
```

Unlike in the previous code, we don't explicitly wait for the transform ahead of time.  However, we do pass the argument `wait_for_frames=True`.  This will cause the lookup to wait until the frames are available.  Without that argument, the listener will throw an exception immediately if the frames are not available (you can try this!).  Generally if you expect frames to be available, you should set `wait_for_frames=False` to avoid your code hanging if you make a typo in frame names.

Note that you can also use a `transform_time` argument to have the TF listener wait for a transform at a particular time.  We do not show that because we use a static transform broadcaster, which means we're broadcasting a transform at all times.


It's also good practice to pass any potentially blocking call the `timeout_sec` argument.  We did not do this here because the code needed to wait on the user starting the static transform publishing, which could be arbitrarily long, but leaving this argument out can lead to infinite hangs if the transforms never become available.
