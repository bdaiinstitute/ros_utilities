import time

import synchros2.process as ros_process
import synchros2.scope as ros_scope
from synchros2.action import Actionable, ActionFuture
from synchros2.futures import wait_for_future
from synchros2_tutorials_interfaces_example.action import Wait

# The message type for feedback callbacks is weird
from synchros2_tutorials_interfaces_example.action._wait import Wait_FeedbackMessage


class WaitClient:
    """Example class that shows how to use synchros2's Actionable with the Wait action."""

    def __init__(self):
        self._node = ros_scope.ensure_node()
        self._wait = Actionable(Wait, "wait_action")

    def initialize(self) -> bool:
        """Waits for the server and returns False if the server is not available within 5s."""
        if not self._wait.wait_for_server(timeout_sec=5.0):
            self._node.get_logger().error("Unable to find server for wait_action")
            return False
        return True

    def request_wait_synchronously(self, n_seconds_to_wait: float) -> Wait.Result | None:
        """Requests a wait synchronously, uses a feedback callback to print feedback, and returns the result."""
        request = Wait.Goal()
        request.n_seconds_to_wait = n_seconds_to_wait
        return self._wait.synchronously(request, feedback_callback=self._print_feedback, nothrow=True)

    def request_wait_asynchronously(self, n_seconds_to_wait: float) -> ActionFuture[Wait.Result, Wait.Feedback]:
        """Requests a wait asynchronously and sets up a callback that prints the result when the action finishes."""
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
    """Main function that shows synchronous and asynchronous action call examples."""
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
