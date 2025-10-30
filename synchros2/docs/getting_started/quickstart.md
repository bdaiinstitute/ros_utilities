
# Quickstart

## Overview

`synchros2` is a framework that allows blocking in callbacks without the user needing to worry about the threading.  The simplest way to use it is to only ever use the node `synchros2` creates (the "ros scope" node) everywhere:

1. Decorate your `main` with the `@ros_process.main()` decorator:
    
    ```python
    import synchros2.process as ros_process
    
    @ros_process.main()
    def main() -> None:
    	...
    ```
    
2. Only use the node associated with `ros_scope`:
    
    ```python
    import synchros2.scope as ros_scope
    
    class MyClass:
    	def __init__(self):
    		self._node = ros_scope.ensure_node()
    ```

## Your First `synchros2` Code

We’ll now create and run example `synchros2` code.  We’ll analyze the code in the next section.

### Interface

We'll start by creating a custom message for us to publish and listen to.

1. Create a package called `synchros2_tutorials_interfaces` for holding our custom messages:
    
    ```bash
    cd <workspace>/src
    ros2 pkg create synchros2_tutorials_interfaces --license MIT
    ```

2. Create a `msg` directory inside the package:

   ```bash
   mkdir -p <workspace>/src/synchros2_tutorials_interfaces/msg
    ```

3. Create a file `<workspace>/src/synchros2_tutorials_interfaces/msg/String.msg` with the following text:

    ```
    string data
    ```
    (This is the same as `std_msgs/String` but it's been deprecated.  Additionally, we'll use this interfaces packages in later tutorials.)

4. Add the following lines to `<workspace>/src/synchros2_tutorials_interfaces/CMakeLists.txt` under the existing `find_package`:

   ```make
   find_package(rosidl_default_generators REQUIRED)

   rosidl_generate_interfaces(${PROJECT_NAME}
     "msg/String.msg"
   )
   ```
   Make sure to capitalize `String.msg`!

5. Update your `package.xml` file to include the dependencies for message generation.  Here is what the full file should look like (you can replace what's there with this entirely):
   ```xml
   <?xml version="1.0"?>
   <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
   <package format="3">
     <name>synchros2_tutorials_interfaces</name>
     <version>0.0.1</version>
     <description>Interface messages for the synchros2 tutorials</description>
     <maintainer email="opensource@rai-inst.com">RAI Institute</maintainer>
     <license>MIT</license>

     <buildtool_depend>ament_cmake</buildtool_depend>
     <buildtool_depend>rosidl_default_generators</buildtool_depend>
     <exec_depend>rosidl_default_runtime</exec_depend>

     <test_depend>ament_lint_auto</test_depend>
     <test_depend>ament_lint_common</test_depend>

     <member_of_group>rosidl_interface_packages</member_of_group>

     <export>
       <build_type>ament_cmake</build_type>
     </export>

   </package>
   ```

5. Build the interfaces packages:

    ```bash
    cd <workspace>
    colcon build --packages-select synchros2_tutorials_interfaces
    source install/setup.bash
    ```

6. Check we're aware of the message by having `ros2 interface` show it:

    ```bash
    ros2 interface show synchros2_tutorials_interfaces/msg/String
    ```
    should print

    ```bash
    string data
    ```
    If `ros2 interface` cannot find the package make sure you:
    * Sourced `install/setup.bash` in your current terminal *after* building.
    * Built from `<workspace>` and not any other directory.

### Writing and Running the Code

We will write our code in a different package from the "interface", i.e. messages, package we just created.  This is good practice in general as it allows people to download the interface to your code without also needing all of the dependencies your code might pull in.  It also works around the fact that ROS2 does not make it easy to have message generation and Python code in the same package.

1. Create a package called `synchros2_tutorials` that depends on `synchros2` and `synchros2_tutorials_interfaces` (by specifying `--dependencies rclpy synchros2 synchros2_tutorials_interfaces` the package creation will automatically add these dependencies to the created `package.xml`): 
    
    ```bash
    cd <workspace>/src
    ros2 pkg create --build-type ament_python synchros2_tutorials  --dependencies rclpy synchros2 synchros2_tutorials_interfaces --license MIT
    ```
    
3. Create the file `<workspace>/src/synchros2_tutorials/synchros2_tutorials/listener.py` and add the following code to it:
    
    ```python
    import time

    import synchros2_tutorials_interfaces.msg

    import synchros2.process as ros_process
    import synchros2.scope as ros_scope


    class Listener:
        """Example class that has two subscribers."""

        def __init__(self):
            self._node = ros_scope.ensure_node()
            self._sub1 = self._node.create_subscription(synchros2_tutorials_interfaces.msg.String, "chat", self._callback1, 1)
            self._sub2 = self._node.create_subscription(synchros2_tutorials_interfaces.msg.String, "chat", self._callback2, 1)
            self._node.get_logger().info("Listening!")

        def _callback1(self, msg: synchros2_tutorials_interfaces.msg.String) -> None:
            self._node.get_logger().info(f"Callback 1 received message {msg} and will now sleep for 10 seconds")
            time.sleep(10)
            self._node.get_logger().info(f"Callback 1 is done sleeping after receiving {msg}")

        def _callback2(self, msg: synchros2_tutorials_interfaces.msg.String) -> None:
            self._node.get_logger().info(f"Callback 2 received message {msg} and will now sleep for 5 seconds")
            time.sleep(5)
            self._node.get_logger().info(f"Callback 2 is done sleeping after receiving {msg}")


    @ros_process.main()
    def main() -> None:
        """Main function that just creates the listener class and waits for Ctrl+C"""
        _ = Listener()
        ros_process.wait_for_shutdown()

    if __name__ == '__main__':
        main()
    ```
    
4. Run the code (make sure you've sourced `<workspace>/install/setup.bash` in your current terminal):
    
    ```bash
    cd <workspace>/src/synchros2_tutorials/synchros2_tutorials
    python listener.py
    ```
    
    You should see the print out `Listening!` 
    
5. Now we’ll publish some messages to the `/chat` topic.  For the best example, create a new terminal to do this (make sure to source `<workspace/install/setup.bash`) and make sure you can see the terminal in which the listener is running.  Run the two commands in quick succession:
    
    ```bash
    ros2 topic pub /chat synchros2_tutorials_interfaces/msg/String "data: 'This is the first message'" -1
    ros2 topic pub /chat synchros2_tutorials_interfaces/msg/String "data: 'This is the second message'" -1
    ```
    
6. The final output in your listener terminal should look something like the following:
    
    ```
    [INFO] [1759264514.587662269] [listener]: Listening!
    [INFO] [1759264519.873956826] [listener]: Callback 1 received message synchros2_tutorials_interfaces.msg.String(data='This is the first message') and will now sleep for 10 seconds
    [INFO] [1759264519.976236531] [listener]: Callback 2 received message synchros2_tutorials_interfaces.msg.String(data='This is the first message') and will now sleep for 5 seconds
    [INFO] [1759264524.997429745] [listener]: Callback 2 is done sleeping after receiving synchros2_tutorials_interfaces.msg.String(data='This is the first message')
    [INFO] [1759264524.998650337] [listener]: Callback 2 received message synchros2_tutorials_interfaces.msg.String(data='This is the second message') and will now sleep for 5 seconds
    [INFO] [1759264529.884469744] [listener]: Callback 1 is done sleeping after receiving synchros2_tutorials_interfaces.msg.String(data='This is the first message')
    [INFO] [1759264529.885723251] [listener]: Callback 1 received message synchros2_tutorials_interfaces.msg.String(data='This is the second message') and will now sleep for 10 seconds
    [INFO] [1759264530.003322694] [listener]: Callback 2 is done sleeping after receiving synchros2_tutorials_interfaces.msg.String(data='This is the second message')
    [INFO] [1759264539.896067303] [listener]: Callback 1 is done sleeping after receiving synchros2_tutorials_interfaces.msg.String(data='This is the second message')
    ```

## Analyzing the Code

Let’s first look at what the `listener.py` code is intended to do and then we’ll show why it outputs what it does.

We start with the imports all `synchros2` code needs (non-main code doesn’t need the `ros_process` one):

```python
import synchros2.process as ros_process
import synchros2.scope as ros_scope
```

We then create a class that stores the ros scope node:

```python
class Listener:
    def __init__(self):
        self._node = ros_scope.ensure_node()
```

The `synchros2` mechanisms create a single node that we want to use everywhere because it's already set up to create all the appropriate threads for any callbacks added to it.  We refer to this single node as the "ros scope node".  For beginning use, the ros scope node is the *only* node you need or should use.  `synchros2` takes care of spinning any callbacks that we attach to this node and spawning whatever threads we need.  This node can be obtained anywhere in the process with the `ros_scope.ensure_node()` call (which will either return the ros scope node or raise a `ValueError` if one was never created - usually because you forgot to decorate your `main` with `ros2_process.main()`).  **Do not create further nodes.**

Side note: We both create a class and store the node as part of the class because this is a similar pattern to how many ROS1 and ROS2 tutorials are written and matches the pattern we expect users will want with more complicated code.  However, we could simply query for the node each time with `ros_scope.ensure_node()` and we could also create the callbacks without a class.  

Once we’ve got the node we attach two callbacks to it that listen for a string on the `/chat` topic.  See [here](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html) for more information about topics.

```python
        self._sub1 = self._node.create_subscription(synchros2_tutorials_interfaces.msg.String, "chat", self._callback1, 1)
        self._sub2 = self._node.create_subscription(synchros2_tutorials_interfaces.msg.String, "chat", self._callback2, 1)
```

The two callbacks both print the message received, then sleep (callback 1 for 10s and callback 2 for 5s) and print again.  The intention of the sleeping here is to show that we can take significant time in both callbacks without affecting the other.  In user code, the sleep would be replaced with processing the message.

```python
    def _callback1(self, msg: synchros2_tutorials_interfaces.msg.String) -> None:
        self._node.get_logger().info(f"Callback 1 received message {msg} and will now sleep for 10 seconds")
        time.sleep(10)
        self._node.get_logger().info(f"Callback 1 is done sleeping after receiving {msg}")

    def _callback2(self, msg: synchros2_tutorials_interfaces.msg.String) -> None:
        self._node.get_logger().info(f"Callback 2 received message {msg} and will now sleep for 5 seconds")
        time.sleep(5)
        self._node.get_logger().info(f"Callback 2 is done sleeping after receiving {msg}")

```

Our main function is decorated with `ros_process.main()`:

```python
@ros_process.main()
def main() -> None:
```

**This decorator is very important!**  This is what creates the ros scope node and the corresponding executor that keeps track of all of the threads.

Inside the main we create an instance of the listener class and then we wait for shutdown.  This leaves our subscriptions running until the program receives `Ctrl+C`:

```python
    _ = Listener()
    ros_process.wait_for_shutdown()
```

`ros_process.wait_for_shutdown` just waits until ROS2 is shut down.  Note that **there is no spin call**.  Spinning happens in the background and is taken care of by the `synchros2` mechanisms.

### Analyzing the Output

Your output should have looked something like:

```
[INFO] [1759264514.587662269] [listener]: Listening!
[INFO] [1759264519.873956826] [listener]: Callback 1 received message synchros2_tutorials_interfaces.msg.String(data='This is the first message') and will now sleep for 10 seconds
[INFO] [1759264519.976236531] [listener]: Callback 2 received message synchros2_tutorials_interfaces.msg.String(data='This is the first message') and will now sleep for 5 seconds
[INFO] [1759264524.997429745] [listener]: Callback 2 is done sleeping after receiving synchros2_tutorials_interfaces.msg.String(data='This is the first message')
[INFO] [1759264524.998650337] [listener]: Callback 2 received message synchros2_tutorials_interfaces.msg.String(data='This is the second message') and will now sleep for 5 seconds
[INFO] [1759264529.884469744] [listener]: Callback 1 is done sleeping after receiving synchros2_tutorials_interfaces.msg.String(data='This is the first message')
[INFO] [1759264529.885723251] [listener]: Callback 1 received message synchros2_tutorials_interfaces.msg.String(data='This is the second message') and will now sleep for 10 seconds
[INFO] [1759264530.003322694] [listener]: Callback 2 is done sleeping after receiving synchros2_tutorials_interfaces.msg.String(data='This is the second message')
[INFO] [1759264539.896067303] [listener]: Callback 1 is done sleeping after receiving synchros2_tutorials_interfaces.msg.String(data='This is the second message')

```

Here’s what happened:

```{mermaid}
    sequenceDiagram
      participant User
      participant Callback1
      participant Callback2

      User->>Callback1: Publish "This is the first message"
      User->>Callback2: Publish "This is the first message"
      Callback1->>Callback1: Print received, sleep 10s
      Callback2->>Callback2: Print received, sleep 5s (runs concurrently)
      User->>Callback1: Publish "This is the second message"
      User->>Callback2: Publish "This is the second message"
      Note over Callback1,Callback2: Both callbacks busy, second message queued
      Callback2->>Callback2: Finish sleep, print done (after 5s)
      Callback2->>Callback2: Print received second message, sleep 5s
      Callback1->>Callback1: Finish sleep, print done (after 10s)
      Callback1->>Callback1: Print received second message, sleep 10s
      Callback2->>Callback2: Finish sleep, print done (after 5s)
      Callback1->>Callback1: Finish sleep, print done (after 10s)
```

1. We published the message “This is the first message” on `/chat`
2. Immediately after 1: That message went into the queues for both callbacks
3. Immediately after 2: Callback 1 received the first message, printed out the info and started to sleep
4. Immediately after 2: *While callback 1 is running*, callback 2 also received the first message, printed out the info and started to sleep
5. Slightly later: We published the message “This is the second message” on `/chat`
6. Immediately after 5: That message went into the queues for both callbacks.  However the callbacks were both busy so nothing happens yet.
7. 5 seconds after 4: Callback 2 finishes sleeping and prints that it is done sleeping after receiving the first message
8. Immediately after 7: Callback 2 is given the second message now that it’s finished with the first and prints that it’s got that
9. 10 seconds after 3: Callback 1 finishes sleeping and prints that it is done sleeping after receiving the first message
10. Immediately after 9: Callback 1 is given the second message now that it’s finished with the first and prints that it’s got that
11. 5 seconds after 7: Callback 2 finishes sleeping and prints that it’s done sleeping after the second message
12. 10 seconds after 10: Callback 1 finishes sleeping and prints that it’s done sleeping after the second message

There’s some important things to notice:

- Both callback 1 and callback 2 run simultaneously even though they are both blocking.  With bare ROS2 this takes a multi-threaded executor and some finessing of the threads but `synchros2` does this natively.  **`synchros2` allows blocking in callbacks.**
- Both callbacks had to finish before they could be called again.  For example, callback 2 doesn’t print that it’s received the second message until it’s finished processing the first.

This behavior can be changed using [callback groups](https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html) as with native ROS2.  `synchros2` makes these choices by default because they match what was done in ROS1 and feels like the most “common sense” solution.

Side note: This is essentially the behavior of ROS2 with a multi-threaded executor and the callbacks each in their own mutually exclusive callback group.  `synchros2` hides the complexities of setting this up.

## Next Steps

You can now try the [tutorials](../tutorials/index.md) or read up our [guides](../guides/index.md).
