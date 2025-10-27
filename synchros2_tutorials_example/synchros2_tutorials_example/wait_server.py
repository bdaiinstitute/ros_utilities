import rclpy
from rclpy.action.server import ServerGoalHandle

import synchros2.process as ros_process
import synchros2.scope as ros_scope
from synchros2.single_goal_action_server import SingleGoalActionServer
from synchros2_tutorials_interfaces_example.action import Wait


class WaitServer:
    """Example class that shows how to create an action server that waits for a requested amount of time."""

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
    """Main that starts the action server and waits for shutdown."""
    _ = WaitServer()
    ros_process.wait_for_shutdown()


if __name__ == "__main__":
    main()
