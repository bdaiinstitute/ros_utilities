import time

import rclpy

import synchros2.process as ros_process
import synchros2.scope as ros_scope
import synchros2_tutorials_interfaces_example.msg


class MsgProcessor:
    """Example class that does work in a subscriber callback."""

    def __init__(self):
        self._node = ros_scope.ensure_node()
        self.processed_msg = None
        self._sub = self._node.create_subscription(
            synchros2_tutorials_interfaces_example.msg.String,
            "chat",
            self._callback,
            1,
        )
        self._node.get_logger().info("Listening!")

    def _callback(self, msg: synchros2_tutorials_interfaces_example.msg.String) -> None:
        self._node.get_logger().info(f"Callback received message {msg} and will now process it")
        start = self._node.get_clock().now()
        time.sleep(5)
        self.processed_msg = f"{msg}: {start} -> {self._node.get_clock().now()}"
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
