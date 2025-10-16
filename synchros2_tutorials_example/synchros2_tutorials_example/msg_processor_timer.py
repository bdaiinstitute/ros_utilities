import threading
import time

import rclpy

import synchros2.process as ros_process
import synchros2.scope as ros_scope
import synchros2_tutorials_interfaces_example.msg


class MsgProcessor:
    """Example class that records a message in the subscription callback and processes it in a timer callback."""

    def __init__(self):
        self._node = ros_scope.ensure_node()
        self._lock = threading.Lock()
        self._raw_msg = None
        self.processed_msg = None
        self._sub = self._node.create_subscription(
            synchros2_tutorials_interfaces_example.msg.String,
            "chat",
            self._sub_callback,
            1,
        )
        self._timer = self._node.create_timer(timer_period_sec=0.1, callback=self._timer_callback)
        self._node.get_logger().info("Listening!")

    def _sub_callback(self, msg: synchros2_tutorials_interfaces_example.msg.String) -> None:
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
    """Main function that prints the processed message once per second."""
    mp = MsgProcessor()
    node = ros_scope.ensure_node()
    rate = node.create_rate(1.0)
    while rclpy.ok():
        node.get_logger().info(f"Processed message is {mp.processed_msg}")
        rate.sleep()


if __name__ == "__main__":
    main()
