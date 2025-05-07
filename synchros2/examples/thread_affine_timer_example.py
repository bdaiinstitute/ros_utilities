# Copyright (c) 2025 Boston Dynamics AI Institute LLC.  All rights reserved.

"""An example of a ROS 2 aware command using process-wide machinery.

Run with:

```sh
python3 examples/thread_affine_timer_example.py
```
"""

import threading

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import synchros2.process as ros_process
from synchros2.executors import AutoScalingMultiThreadedExecutor, foreground
from synchros2.node import Node


@ros_process.main(prebaked=False, interruptible=True)
def main() -> None:
    """Example entrypoint.

    It is first configured as a regular ROS 2 aware process, but process-wide
    (i.e. globally accessible) multi-threaded executor and node are set immediately
    on start. Still, automatic ``logging`` logs forwarding to the ROS 2 logging
    system is disabled. Implicit namespacing is also disabled.

    When executed, two timers are created on the process-wide node: the first timer
    is assigned to a user-defined callback group that is bound to a static thread pool
    with a single thread, whereas the second timer is bound to the default thread pool.
    Idle time for the default thread pool is set such that the second timer is likely
    to run each time on a new thread. Timer callbacks will print the ID of the thread
    they are run by. This will continue indefinitely until the executable is interrupted
    (e.g. by a SIGINT on Ctrl + C).
    """
    with foreground(AutoScalingMultiThreadedExecutor(max_thread_idle_time=0.5)) as main.executor:  # noqa: SIM117
        with main.managed(Node, "example_node") as main.node:
            thread_pool = main.executor.add_static_thread_pool(1)
            custom_callback_group = MutuallyExclusiveCallbackGroup()
            main.executor.bind(custom_callback_group, thread_pool)

            def thread_affine_timer_callback():
                current_thread = threading.current_thread()
                print(f"Timer 1 is affine to {current_thread.name} ({current_thread.ident})")

            main.node.create_timer(1.0, thread_affine_timer_callback, custom_callback_group)

            def timer_callback():
                current_thread = threading.current_thread()
                print(f"Timer 2 run by {current_thread.name} ({current_thread.ident})")

            main.node.create_timer(1.0, timer_callback)

            main.executor.spin()


if __name__ == "__main__":
    main()
