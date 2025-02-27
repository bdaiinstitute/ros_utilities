# Copyright (c) 2023 Boston Dynamics AI Institute LLC.  All rights reserved.

"""An example of a ROS 2 aware executable using process-wide machinery.

Run with:

```sh
python3 examples/background_action_example.py
```

And follow the instructions on screen.
"""

import sys
from typing import Any

from example_interfaces.action import Fibonacci
from rclpy.action.client import ActionClient
from rclpy.action.server import ActionServer, ServerGoalHandle

import synchros2.process as ros_process
from synchros2.node import Node


class MinimalActionServer(Node):
    """A minimal ROS 2 node serving an example_interfaces.action.Fibonacci action."""

    def __init__(self, node_name: str = "minimal_action_server", **kwargs: Any) -> None:
        super().__init__(node_name, **kwargs)
        self._action_server = ActionServer(self, Fibonacci, "compute_fibonacci_sequence", self.execute_callback)

    def execute_callback(self, goal_handle: ServerGoalHandle) -> Fibonacci.Result:
        sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            sequence.append(sequence[i] + sequence[i - 1])
            feedback = Fibonacci.Feedback()
            feedback.sequence = sequence
            goal_handle.publish_feedback(feedback)
        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = sequence
        return result


@ros_process.main()
def main() -> None:
    """Example entrypoint.

    It is configured almost as a prebaked ROS 2 aware process. That is, including a
    process-wide (ie. globally accessible) node, an autoscaling multi-threaded executor
    running in a background thread, automatic ``logging`` logs forwarding to the ROS 2
    logging system, and implicit namespacing for all ROS 2 nodes loaded in it based on
    the executable basename. A convenience for simple executables that need to use ROS 2
    without having to worry about ROS 2 (or its fallout, e.g. accidental deadblocking).

    When executed, an action server node is loaded, and the process-wide node is used to
    setup an action client. This action client is then used to implement a simple console
    application.
    """
    main.load(MinimalActionServer)

    action_client = ActionClient(main.node, Fibonacci, "compute_fibonacci_sequence")
    assert action_client.wait_for_server(timeout_sec=5)

    while main.context.ok():
        line = input("Please provide a Fibonacci sequence order (or press Enter to exit): ")
        if not line:
            break
        try:
            order = int(line)
        except ValueError:
            print(f"    Hmm, '{line}' is not a number", file=sys.stderr)
            continue
        result = action_client.send_goal(Fibonacci.Goal(order=order))
        print("Computed Fibonacci sequence: ", list(result.result.sequence))
    print("Bye bye!")


if __name__ == "__main__":
    main()
