# Timers and Callbacks

## Prerequisites

Make sure you’ve done the [Getting Started Guide](./GettingStartedGuide.md).

## Introduction

In this tutorial we’ll work through two ways of using `synchros2` for doing work on receipt of a message.  In particular, we’ll assume that each time we receive a `String` message we want to sleep for 5 seconds and then set a class variable (the sleeping is a placeholder for whatever complicated processing might need to be done for a message).  We’ll do this two ways:

1. Using a message callback: The simpler version of this is to just do the work in a subscriber callback.  ROS2 discourages this because of the difficulty of blocking in callbacks in native ROS2 but it’s fine using `synchros2`.
2. Using a message callback and timer: The more classic “ROS2” approach is to do only quick work in the callback and use a timer to constantly check whether a message has been received.

## Doing Work in a Message Callback

### Writing and Running the Code

We’ll start by writing and running the code and then we’ll analyze it.

1. As part of the Getting Started Guide, you should have created a `synchros2_tutorials` packages.  We’ll add this file to it in `<workspace>/src/synchros2_tutorials/synchros2_tutorials/msg_processor_cb.py` and add the following code to it: 
    
    ```python
    import time

    import rclpy
    import synchros2_tutorials_interfaces.msg

    import synchros2.process as ros_process
    import synchros2.scope as ros_scope


    class MsgProcessor:
        def __init__(self):
            self._node = ros_scope.ensure_node()
            self.processed_msg = None
            self._sub = self._node.create_subscription(synchros2_tutorials_interfaces.msg.String, "chat", self._callback, 1)
            self._node.get_logger().info("Listening!")

        def _callback(self, msg: synchros2_tutorials_interfaces.msg.String) -> None:
            self._node.get_logger().info(f"Callback received message {msg} and will now process it")
            start = self._node.get_clock().now()
            time.sleep(5)
            self.processed_msg = f"{msg}: {start} -> {self._node.get_clock().now()}"
            self._node.get_logger().info(f"Set processed message to {self.processed_msg}")


    @ros_process.main()
    def main() -> None:
        mp = MsgProcessor()
        node = ros_scope.ensure_node()
        rate = node.create_rate(1.0)
        while rclpy.ok():
            node.get_logger().info(f"Processed message is {mp.processed_msg}")
            rate.sleep()

    if __name__ == "__main__":
        main()
    ```
    
2. Run the code (make sure you've sourced `<workspace>/install/setup.bash` in this terminal):
    
    ```bash
    cd <workspace>/src/synchros2_tutorials/synchros2_tutorials
    python msg_processor_cb.py
    ```
    
    You should see the print out `Listening!` and then start to print `Processed message is None`.
    
3. Now we’ll publish some messages to the `/chat` topic on which the subscriber is listening.  For the best example, create a new terminal to do this (make sure to source `<workspace>/install/setup.bash) and make sure you can see the terminal in which the listener is running.  Run the publish command:
    
    ```bash
    ros2 topic pub /chat synchros2_tutorials_interfaces/msg/String "data: 'This is a message to process'" -1
    ```
    
4. You should see the processor receive the message and take some time to process it. Once it has the class variable changes: 
    
    ```bash
    [INFO] [1759859896.546288649] [msg_processor_cb]: Processed message is None
    [INFO] [1759859897.043449521] [msg_processor_cb]: Callback received message synchros2_tutorials_interfaces.msg.String(data='This is a message to process') and will now process it
    [INFO] [1759859897.547909617] [msg_processor_cb]: Processed message is None
    [INFO] [1759859898.549327163] [msg_processor_cb]: Processed message is None
    [INFO] [1759859899.550741444] [msg_processor_cb]: Processed message is None
    [INFO] [1759859900.552368980] [msg_processor_cb]: Processed message is None
    [INFO] [1759859901.553779861] [msg_processor_cb]: Processed message is None
    [INFO] [1759859902.049583632] [msg_processor_cb]: Set processed message to synchros2_tutorials_interfaces.msg.String(data='This is a message to process'): Time(nanoseconds=1759859897043782082, clock_type=ROS_TIME) -> Time(nanoseconds=1759859902049301209, clock_type=ROS_TIME)
    [INFO] [1759859902.555178622] [msg_processor_cb]: Processed message is synchros2_tutorials_interfaces.msg.String(data='This is a message to process'): Time(nanoseconds=1759859897043782082, clock_type=ROS_TIME) -> Time(nanoseconds=1759859902049301209, clock_type=ROS_TIME)
    ```
    
5. We can publish another message: 
    
    ```bash
    ros2 topic pub /chat synchros2_tutorials_interfaces/msg/String "data: 'This is another message to process'" -1
    ```
    
6. And we once again see the message processed and changed: 
    
    ```bash
    [INFO] [1759859908.563639805] [msg_processor_cb]: Processed message is synchros2_tutorials_interfaces.msg.String(data='This is a message to process'): Time(nanoseconds=1759859897043782082, clock_type=ROS_TIME) -> Time(nanoseconds=1759859902049301209, clock_type=ROS_TIME)
    [INFO] [1759859908.676727569] [msg_processor_cb]: Callback received message synchros2_tutorials_interfaces.msg.String(data='This is another message to process') and will now process it
    [INFO] [1759859909.565078282] [msg_processor_cb]: Processed message is synchros2_tutorials_interfaces.msg.String(data='This is a message to process'): Time(nanoseconds=1759859897043782082, clock_type=ROS_TIME) -> Time(nanoseconds=1759859902049301209, clock_type=ROS_TIME)
    [INFO] [1759859910.566355131] [msg_processor_cb]: Processed message is synchros2_tutorials_interfaces.msg.String(data='This is a message to process'): Time(nanoseconds=1759859897043782082, clock_type=ROS_TIME) -> Time(nanoseconds=1759859902049301209, clock_type=ROS_TIME)
    [INFO] [1759859911.567687561] [msg_processor_cb]: Processed message is synchros2_tutorials_interfaces.msg.String(data='This is a message to process'): Time(nanoseconds=1759859897043782082, clock_type=ROS_TIME) -> Time(nanoseconds=1759859902049301209, clock_type=ROS_TIME)
    [INFO] [1759859912.569292011] [msg_processor_cb]: Processed message is synchros2_tutorials_interfaces.msg.String(data='This is a message to process'): Time(nanoseconds=1759859897043782082, clock_type=ROS_TIME) -> Time(nanoseconds=1759859902049301209, clock_type=ROS_TIME)
    [INFO] [1759859913.571042294] [msg_processor_cb]: Processed message is synchros2_tutorials_interfaces.msg.String(data='This is a message to process'): Time(nanoseconds=1759859897043782082, clock_type=ROS_TIME) -> Time(nanoseconds=1759859902049301209, clock_type=ROS_TIME)
    [INFO] [1759859913.682406886] [msg_processor_cb]: Set processed message to synchros2_tutorials_interfaces.msg.String(data='This is another message to process'): Time(nanoseconds=1759859908676838909, clock_type=ROS_TIME) -> Time(nanoseconds=1759859913682187744, clock_type=ROS_TIME)
    [INFO] [1759859914.572430338] [msg_processor_cb]: Processed message is synchros2_tutorials_interfaces.msg.String(data='This is another message to process'): Time(nanoseconds=1759859908676838909, clock_type=ROS_TIME) -> Time(nanoseconds=1759859913682187744, clock_type=ROS_TIME)
    [INFO] [1759859915.573779147] [msg_processor_cb]: Processed message is synchros2_tutorials_interfaces.msg.String(data='This is another message to process'): Time(nanoseconds=1759859908676838909, clock_type=ROS_TIME) -> Time(nanoseconds=1759859913682187744, clock_type=ROS_TIME)
    
    ```
    

### Understanding the Code

This code is similar to the code from the [Getting Started Guide](./GettingStartedGuide.md) so we will not go through it in as much detail.

We start with the imports all `synchros2` code needs (non-main code doesn’t need the `ros_process` one):

```python
import synchros2.process as ros_process
import synchros2.scope as ros_scope
```

We then create a class that stores the ros scope node, sets up a callback on the topic `/chat`, and creates a class variable `processed_msg` that we’ll use to store data computed in the callback.  Note that we use our ros scope node for the subscription - this is what lets us do work in the callback without blocking.

```python
class MsgProcessor:
    def __init__(self):
        self._node = ros_scope.ensure_node()
        self._sub = self._node.create_subscription(synchros2_tutorials_interfaces.msg.String, "chat", self._callback, 1)
        self.processed_msg = None
        self._node.get_logger().info("Listening!")

```

The callback receives the message, does some “work” (the sleep here is just a demonstration to show we can block in the callback) and then updates the class variable:

```python
    def _callback(self, msg: synchros2_tutorials_interfaces.msg.String) -> None:
        self._node.get_logger().info(f"Callback received message {msg} and will now process it")
        start = self._node.get_clock().now()
        time.sleep(5)
        self.processed_msg = f"{msg}: {start} -> {self._node.get_clock().now()}"
        self._node.get_logger().info(f"Set processed message to {self.processed_msg}")

```

As with all `synchros2` code, our main function is decorated with `ros_process.main()` :

```python
@ros_process.main()
def main() -> None:
```

Inside the main we create our message processor:
```python
        mp = MsgProcessor()
```

We want to print out the processed message once per second so we create a `Rate` that lets us do that.  The reason to use `Rate` and not just `time.sleep` is that the `Rate` respects ros time (e.g. simulations running slower than real time):
```python
        rate = node.create_rate(1.0)
```

We loop only while `rclpy.ok()` so that the program will die on Ctrl+C:
```python
        while rclpy.ok():
            node.get_logger().info(f"Processed message is {mp.processed_msg}")
            rate.sleep()
```

## Doing Work with Timers

Sometimes, especially when there are multiple subscriptions involved, it’s better to do work in a single timer callback rather than in multiple subscription callbacks.  Here we show how to accomplish the same task as before but this time using the callback and timer approach.

### Writing and Running the Code

1. Add the following to `<workspace>/src/synchros2_tutorials/synchros2_tutorials/msg_processor_timer.py` : 
    
    ```python
    import threading
    import time

    import rclpy
    import synchros2_tutorials_interfaces.msg

    import synchros2.process as ros_process
    import synchros2.scope as ros_scope


    class MsgProcessor:
        def __init__(self):
            self._node = ros_scope.ensure_node()
            self._lock = threading.Lock()
            self._raw_msg = None
            self.processed_msg = None
            self._sub = self._node.create_subscription(synchros2_tutorials_interfaces.msg.String, "chat", self._sub_callback, 1)
            self._timer = self._node.create_timer(timer_period_sec=0.1, callback=self._timer_callback)
            self._node.get_logger().info("Listening!")

        def _sub_callback(self, msg: synchros2_tutorials_interfaces.msg.String) -> None:
            self._node.get_logger().info(f"Callback received message {msg}")
            # Store the message
            with self._lock:
                self._raw_msg = msg

        def _timer_callback(self) -> None:
            # Check if we have a new message
            with self._lock:
                if self._raw_msg is None:
                    return
                new_msg = self._raw_msg
                self._raw_msg = None
            start = self._node.get_clock().now()
            time.sleep(5)
            self.processed_msg = f"{new_msg}: {start} -> {self._node.get_clock().now()}"
            self._node.get_logger().info(f"Set processed message to {self.processed_msg}")


    @ros_process.main()
    def main() -> None:
        mp = MsgProcessor()
        node = ros_scope.ensure_node()
        rate = node.create_rate(1.0)
        while rclpy.ok():
            node.get_logger().info(f"Processed message is {mp.processed_msg}")
            rate.sleep()


    if __name__ == "__main__":
        main()
    ```
    
2. Run the code:
    
    ```bash
    cd <workspace>/src/synchros2_tutorials/synchros2_tutorials
    python msg_processor_timer.py
    ```
    
    You should see the print out `Listening!` and then start to print `Processed message is None` just as before.
    
3. Now we’ll publish some messages to the `/chat` topic on which the subscriber is listening.  For the best example, create a new terminal to do this and make sure you can see the terminal in which the listener is running.  Run the publish command:
    
    ```bash
    ros2 topic pub /chat synchros2_tutorials_interfaces/msg/String "data: 'This is a message to process'" -1
    ```
    
4. You should see the processor receive the message and take some time to process it. Once it has the class variable changes and we see the same output as before: 
    
    ```bash
    [INFO] [1759861374.357112381] [msg_processor_timer]: Processed message is None
    [INFO] [1759861374.483683352] [msg_processor_timer]: Callback received message synchros2_tutorials_interfaces.msg.String(data='This is a message to process')
    [INFO] [1759861375.358591888] [msg_processor_timer]: Processed message is None
    [INFO] [1759861376.360284475] [msg_processor_timer]: Processed message is None
    [INFO] [1759861377.361184952] [msg_processor_timer]: Processed message is None
    [INFO] [1759861378.362432461] [msg_processor_timer]: Processed message is None
    [INFO] [1759861379.363776974] [msg_processor_timer]: Processed message is None
    [INFO] [1759861379.591944934] [msg_processor_timer]: Set processed message to synchros2_tutorials_interfaces.msg.String(data='This is a message to process'): Time(nanoseconds=1759861374586522303, clock_type=ROS_TIME) -> Time(nanoseconds=1759861379591744376, clock_type=ROS_TIME)
    [INFO] [1759861380.365174121] [msg_processor_timer]: Processed message is synchros2_tutorials_interfaces.msg.String(data='This is a message to process'): Time(nanoseconds=1759861374586522303, clock_type=ROS_TIME) -> Time(nanoseconds=1759861379591744376, clock_type=ROS_TIME)
    
    ```
    
5. We can publish another message: 
    
    ```bash
    ros2 topic pub /chat synchros2_tutorials_interfaces/msg/String "data: 'This is anothe message to process'" -1
    ```
    
6. And we once again see the same output we did above: 
    
    ```bash
    [INFO] [1759861385.371393780] [msg_processor_timer]: Processed message is synchros2_tutorials_interfaces.msg.String(data='This is a message to process'): Time(nanoseconds=1759861374586522303, clock_type=ROS_TIME) -> Time(nanoseconds=1759861379591744376, clock_type=ROS_TIME)
    [INFO] [1759861386.355815066] [msg_processor_timer]: Callback received message synchros2_tutorials_interfaces.msg.String(data='This is another message to process')
    [INFO] [1759861386.372675156] [msg_processor_timer]: Processed message is synchros2_tutorials_interfaces.msg.String(data='This is a message to process'): Time(nanoseconds=1759861374586522303, clock_type=ROS_TIME) -> Time(nanoseconds=1759861379591744376, clock_type=ROS_TIME)
    [INFO] [1759861387.374007289] [msg_processor_timer]: Processed message is synchros2_tutorials_interfaces.msg.String(data='This is a message to process'): Time(nanoseconds=1759861374586522303, clock_type=ROS_TIME) -> Time(nanoseconds=1759861379591744376, clock_type=ROS_TIME)
    [INFO] [1759861388.375589460] [msg_processor_timer]: Processed message is synchros2_tutorials_interfaces.msg.String(data='This is a message to process'): Time(nanoseconds=1759861374586522303, clock_type=ROS_TIME) -> Time(nanoseconds=1759861379591744376, clock_type=ROS_TIME)
    [INFO] [1759861389.376926446] [msg_processor_timer]: Processed message is synchros2_tutorials_interfaces.msg.String(data='This is a message to process'): Time(nanoseconds=1759861374586522303, clock_type=ROS_TIME) -> Time(nanoseconds=1759861379591744376, clock_type=ROS_TIME)
    [INFO] [1759861390.378436138] [msg_processor_timer]: Processed message is synchros2_tutorials_interfaces.msg.String(data='This is a message to process'): Time(nanoseconds=1759861374586522303, clock_type=ROS_TIME) -> Time(nanoseconds=1759861379591744376, clock_type=ROS_TIME)
    [INFO] [1759861391.379935569] [msg_processor_timer]: Processed message is synchros2_tutorials_interfaces.msg.String(data='This is a message to process'): Time(nanoseconds=1759861374586522303, clock_type=ROS_TIME) -> Time(nanoseconds=1759861379591744376, clock_type=ROS_TIME)
    [INFO] [1759861391.458151208] [msg_processor_timer]: Set processed message to synchros2_tutorials_interfaces.msg.String(data='This is another message to process'): Time(nanoseconds=1759861386452594019, clock_type=ROS_TIME) -> Time(nanoseconds=1759861391457807342, clock_type=ROS_TIME)
    [INFO] [1759861392.380847157] [msg_processor_timer]: Processed message is synchros2_tutorials_interfaces.msg.String(data='This is another message to process'): Time(nanoseconds=1759861386452594019, clock_type=ROS_TIME) -> Time(nanoseconds=1759861391457807342, clock_type=ROS_TIME)
    [INFO] [1759861393.381200862] [msg_processor_timer]: Processed message is synchros2_tutorials_interfaces.msg.String(data='This is another message to process'): Time(nanoseconds=1759861386452594019, clock_type=ROS_TIME) -> Time(nanoseconds=1759861391457807342, clock_type=ROS_TIME)
    ```
    

### Understanding the Code

As always we start with the imports all synchros2 code needs (non-main code doesn’t need the `ros_process` one):

```python
import synchros2.process as ros_process
import synchros2.scope as ros_scope
```

We now create a class and store the ros scope node for easy use:

```python
class MsgProcessor:
    def __init__(self):
        self._node = ros_scope.ensure_node()
```

We then create the data we are going to need:

```python
        self._lock = threading.Lock()
        self._raw_msg = None
        self.processed_msg = None
```

Because we have two callbacks in play, we need a little more data here.  Specifically, we’re going to need to store the message that comes in (`self._raw_msg`).  We also need to make sure that we aren’t trying to read and write to `self._raw_msg` at the same time which is where the lock comes in.  Note that we create the data that the callbacks need *before* we create the callbacks thus avoiding any chance the callbacks get called before that data is available.

We create the subscription to `/chat` like normal using our ros scope node:

```python
        self._sub = self._node.create_subscription(synchros2_tutorials_interfaces.msg.String, "chat", self._sub_callback, 1)
```

But this time this callback will just store the message and return quickly. 

However we then create *another* callback:

```python
        self._timer = self._node.create_timer(timer_period_sec=0.1, callback=self._timer_callback)

```

This creates a timer callback that will be called every 0.1s or as soon as the previous timer callback finished whichever happens last.  This callback is where we’ll do our actual work.

The subscription callback stores the raw message:

```python
    def _sub_callback(self, msg: synchros2_tutorials_interfaces.msg.String) -> None:
        self._node.get_logger().info(f"Callback received message {msg}")
        # Store the message
        with self._lock:
            self._raw_msg = msg
```

Note the use of the lock to make sure that it isn’t writing to `self._raw_msg` while timer callback is trying to read from it.

The real work happens in the timer callback.  The first thing it does is check if there is a new message since the last time the timer was called

```python
    def _timer_callback(self) -> None:
        # Check if we have a new message
        with self._lock:
            if self._raw_msg is None:
                return
```

If `raw_msg` is `None` that means the subscription callback hasn’t run and assigned it to anything so we can return.  Note the use of the lock to ensure that we’re not reading from `raw_msg` while the subscription callback is copying to it.  By using `with` we ensure that the lock is released whenever we exit from the scope of the `with` either by returning or reaching the end of the `with` statement.

If there is a new message we store that message locally so that we can release the lock:

```python
            new_msg = self._raw_msg
            self._raw_msg = None
```

We also reset `self._raw_msg` to `None`.   This means that the next time we go through the timer callback, we won’t process the same message again.

The rest is the same as for the simple callback case.

### Analyzing the Output

This one is a little more complicated than the simple callback so let’s analyze what happened when we published our messages:

1. Before publishing: Timer callback fires every 0.1s.  `raw_msg` is always `None` so it just shortcuts out
2. First message published: `_sub_callback` is called, prints, and assigns `raw_msg` to the first message
3. Within 0.1s after 2:  The timer fires and calls `_timer_callback`.  It sees that `raw_msg` is not `None` , copies it locally, sets it back to `None` and begins processing it.  During this time the `processed_msg` class variable remains `None` so the main loop is still printing `None`
4. Second message published: `_sub_callback` is called, prints, and assigns `raw_msg` to the second message
5. 5 seconds after 3: The timer callback finishes processing the first message, assigns `processed_msg`, and prints.  The main loop now starts printing the processed first message.
6. Immediately after 5: The timer callback fires again because it’s been queued up since 0.1s after 3.  It processes the second message.
7. 5 seconds after 6: The timer callback finishes processing the second message and we see the new prints.

## Further Thoughts

### Should I Do Work in a Subscription Callback or a Timer Callback?

It’s really up to you.  In the case we have here where we have only a single subscription, it’s probably simplest to do the work in the subscription callback.  However if you have multiple subscriptions then it might be easier to process all of the messages in a single, central callback.

### Can We Miss Messages?

If you read the timer callback analysis carefully, you might have noticed it’s possible to miss processing messages.  In fact, both of these approaches as written have this problem.

Let’s start by showing the simple callback approach will miss messages.  To make this easier, change the `time.sleep` in `_callback` in `msg_processor_cb.py` from 5 to 10 seconds to give yourself enough time to publish messages.  Now in quick succession publish 3 messages (it’s important that the third message publish before the first message has finished processing):

```bash
ros2 topic pub /chat synchros2_tutorials_interfaces/msg/String "data: 'This is a message to process'" -1
ros2 topic pub /chat synchros2_tutorials_interfaces/msg/String "data: 'This is another message to process'" -1
ros2 topic pub /chat synchros2_tutorials_interfaces/msg/String "data: 'This is a third message to process'" -1
```

Here’s is approximately the output you’ll get:

```bash
[INFO] [1759872431.388333426] [msg_processor_cb]: Processed message is None
[INFO] [1759872431.885969597] [msg_processor_cb]: Callback received message synchros2_tutorials_interfaces.msg.String(data='This is a message to process') and will now process it
[INFO] [1759872432.388881060] [msg_processor_cb]: Processed message is None
[INFO] [1759872433.389978026] [msg_processor_cb]: Processed message is None
[INFO] [1759872434.391377285] [msg_processor_cb]: Processed message is None
[INFO] [1759872435.394967942] [msg_processor_cb]: Processed message is None
[INFO] [1759872436.399855245] [msg_processor_cb]: Processed message is None
[INFO] [1759872437.400938552] [msg_processor_cb]: Processed message is None
[INFO] [1759872438.402854894] [msg_processor_cb]: Processed message is None
[INFO] [1759872439.405090330] [msg_processor_cb]: Processed message is None
[INFO] [1759872440.422442177] [msg_processor_cb]: Processed message is None
[INFO] [1759872441.424259889] [msg_processor_cb]: Processed message is None
[INFO] [1759872441.896979709] [msg_processor_cb]: Set processed message to synchros2_tutorials_interfaces.msg.String(data='This is a message to process'): Time(nanoseconds=1759872431886114566, clock_type=ROS_TIME) -> Time(nanoseconds=1759872441896735176, clock_type=ROS_TIME)
[INFO] [1759872441.898016455] [msg_processor_cb]: Callback received message synchros2_tutorials_interfaces.msg.String(data='This is a third message to process') and will now process it
[INFO] [1759872442.425762397] [msg_processor_cb]: Processed message is synchros2_tutorials_interfaces.msg.String(data='This is a message to process'): Time(nanoseconds=1759872431886114566, clock_type=ROS_TIME) -> Time(nanoseconds=1759872441896735176, clock_type=ROS_TIME)

```

We never see the callback receive `This is another message to process`!  This is because we’ve set our subscription queue size to 1 when we created our callback (the last argument passed):

```python
        self._sub = self._node.create_subscription(synchros2_tutorials_interfaces.msg.String, "chat", self._callback, 1)

```

So the subscription will only hold on to one message for us.  When the third message is published, the subscription drops the second.  If you increase the queue size to 2 (or more) and run the same experiment, you’ll see all 3 messages get processed:

```python
        self._sub = self._node.create_subscription(synchros2_tutorials_interfaces.msg.String, "chat", self._callback, 2)

```

So it’s important to pay attention to queue size for subscribers if you want to make sure you see every message!  (And this is in fact one argument for not doing a lot of work in subscribers.)

The timer approach will also miss the second message in this same circumstance.  To see this increase the sleep time in `_timer_callback` in `msg_processor_timer.py` from 5 seconds to 10 seconds and once again publish 3 messages in as quick succession as you can:

```bash
ros2 topic pub /chat synchros2_tutorials_interfaces/msg/String "data: 'This is a message to process'" -1
ros2 topic pub /chat synchros2_tutorials_interfaces/msg/String "data: 'This is another message to process'" -1
ros2 topic pub /chat synchros2_tutorials_interfaces/msg/String "data: 'This is a third message to process'" -1
```

Here is the approximate output:

```bash
[INFO] [1759871770.309911305] [msg_processor_timer]: Processed message is None
[INFO] [1759871770.738401897] [msg_processor_timer]: Callback received message synchros2_tutorials_interfaces.msg.String(data='This is a message to process')
[INFO] [1759871771.311272017] [msg_processor_timer]: Processed message is None
[INFO] [1759871772.311959109] [msg_processor_timer]: Processed message is None
[INFO] [1759871773.312986436] [msg_processor_timer]: Processed message is None
[INFO] [1759871774.313934173] [msg_processor_timer]: Processed message is None
[INFO] [1759871775.315538334] [msg_processor_timer]: Processed message is None
[INFO] [1759871775.476007255] [msg_processor_timer]: Callback received message synchros2_tutorials_interfaces.msg.String(data='This is another message to process')
[INFO] [1759871776.317027063] [msg_processor_timer]: Processed message is None
[INFO] [1759871777.318399276] [msg_processor_timer]: Processed message is None
[INFO] [1759871778.318939816] [msg_processor_timer]: Processed message is None
[INFO] [1759871778.469196545] [msg_processor_timer]: Callback received message synchros2_tutorials_interfaces.msg.String(data='This is a third message to process')
[INFO] [1759871779.319936086] [msg_processor_timer]: Processed message is None
[INFO] [1759871780.320987484] [msg_processor_timer]: Processed message is None
[INFO] [1759871780.852170291] [msg_processor_timer]: Set processed message to synchros2_tutorials_interfaces.msg.String(data='This is a message to process'): Time(nanoseconds=1759871770841755415, clock_type=ROS_TIME) -> Time(nanoseconds=1759871780851964183, clock_type=ROS_TIME)
[INFO] [1759871781.321943060] [msg_processor_timer]: Processed message is synchros2_tutorials_interfaces.msg.String(data='This is a message to process'): Time(nanoseconds=1759871770841755415, clock_type=ROS_TIME) -> Time(nanoseconds=1759871780851964183, clock_type=ROS_TIME)
[INFO] [1759871782.323060836] [msg_processor_timer]: Processed message is synchros2_tutorials_interfaces.msg.String(data='This is a message to process'): Time(nanoseconds=1759871770841755415, clock_type=ROS_TIME) -> Time(nanoseconds=1759871780851964183, clock_type=ROS_TIME)
[INFO] [1759871783.324490200] [msg_processor_timer]: Processed message is synchros2_tutorials_interfaces.msg.String(data='This is a message to process'): Time(nanoseconds=1759871770841755415, clock_type=ROS_TIME) -> Time(nanoseconds=1759871780851964183, clock_type=ROS_TIME)
[INFO] [1759871784.324976434] [msg_processor_timer]: Processed message is synchros2_tutorials_interfaces.msg.String(data='This is a message to process'): Time(nanoseconds=1759871770841755415, clock_type=ROS_TIME) -> Time(nanoseconds=1759871780851964183, clock_type=ROS_TIME)
[INFO] [1759871785.326381424] [msg_processor_timer]: Processed message is synchros2_tutorials_interfaces.msg.String(data='This is a message to process'): Time(nanoseconds=1759871770841755415, clock_type=ROS_TIME) -> Time(nanoseconds=1759871780851964183, clock_type=ROS_TIME)
[INFO] [1759871786.327914388] [msg_processor_timer]: Processed message is synchros2_tutorials_interfaces.msg.String(data='This is a message to process'): Time(nanoseconds=1759871770841755415, clock_type=ROS_TIME) -> Time(nanoseconds=1759871780851964183, clock_type=ROS_TIME)
[INFO] [1759871787.329321320] [msg_processor_timer]: Processed message is synchros2_tutorials_interfaces.msg.String(data='This is a message to process'): Time(nanoseconds=1759871770841755415, clock_type=ROS_TIME) -> Time(nanoseconds=1759871780851964183, clock_type=ROS_TIME)
[INFO] [1759871788.330750073] [msg_processor_timer]: Processed message is synchros2_tutorials_interfaces.msg.String(data='This is a message to process'): Time(nanoseconds=1759871770841755415, clock_type=ROS_TIME) -> Time(nanoseconds=1759871780851964183, clock_type=ROS_TIME)
[INFO] [1759871789.331937502] [msg_processor_timer]: Processed message is synchros2_tutorials_interfaces.msg.String(data='This is a message to process'): Time(nanoseconds=1759871770841755415, clock_type=ROS_TIME) -> Time(nanoseconds=1759871780851964183, clock_type=ROS_TIME)
[INFO] [1759871790.332994130] [msg_processor_timer]: Processed message is synchros2_tutorials_interfaces.msg.String(data='This is a message to process'): Time(nanoseconds=1759871770841755415, clock_type=ROS_TIME) -> Time(nanoseconds=1759871780851964183, clock_type=ROS_TIME)
[INFO] [1759871790.864175154] [msg_processor_timer]: Set processed message to synchros2_tutorials_interfaces.msg.String(data='This is a third message to process'): Time(nanoseconds=1759871780853744808, clock_type=ROS_TIME) -> Time(nanoseconds=1759871790863957273, clock_type=ROS_TIME)
[INFO] [1759871791.334280919] [msg_processor_timer]: Processed message is synchros2_tutorials_interfaces.msg.String(data='This is a third message to process'): Time(nanoseconds=1759871780853744808, clock_type=ROS_TIME) -> Time(nanoseconds=1759871790863957273, clock_type=ROS_TIME)

```

Unlike in the case where we did the work in the callback, this time the callback receives all 3 messages even though the queue size is 1.  This is because we return quickly from the callback.  However we still never process the second message.  Here’s what happens:

1. Before publishing: Timer callback fires every 0.1s.  `raw_msg` is always `None` so it just shortcuts out
2. First message published: `_sub_callback` is called, prints, and assigns `raw_msg` to the first message
3. Within 0.1s after 2: The timer fires and `_timer_callback` is called.  It sets `_raw_msg` to `None` and starts processing the first message.
4. Second message published: `_sub_callback` is called, prints, and assigns `raw_msg` to the second message.  However the timer callback doesn’t fire because it’s still busy with the first message.
5. Third message published: `_sub_callback` is called, prints, and assigns `raw_msg` to the third message.  
6. 10 seconds after 3: The timer callback finishes processing the first message, assigns `processed_msg`, and prints.  The main loop now starts printing the processed first message.
7. Immediately after 6: The timer callback fires again because it’s been queued up since 0.1s after 3.  It processes the message stored in `_raw_msg` which is the *third* message not the second.
8. 10 seconds after 6: The timer callback finishes processing the third message and we see the new prints.

So we once again fail to process the second message.  This could be addressed by using a queue for `_raw_msg`.
