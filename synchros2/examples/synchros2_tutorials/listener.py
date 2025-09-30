import time

import std_msgs.msg

import synchros2.process as ros_process
import synchros2.scope as ros_scope


class Listener:
    def __init__(self):
        self._node = ros_scope.ensure_node()
        self._sub1 = self._node.create_subscription(std_msgs.msg.String, "chat", self._callback1, 1)
        self._sub2 = self._node.create_subscription(std_msgs.msg.String, "chat", self._callback2, 1)
        self._node.get_logger().info("Listening!")

    def _callback1(self, msg: std_msgs.msg.String) -> None:
        self._node.get_logger().info(f"Callback 1 received message {msg} and will now sleep for 10 seconds")
        time.sleep(10)
        self._node.get_logger().info(f"Callback 1 is done sleeping after receiving {msg}")

    def _callback2(self, msg: std_msgs.msg.String) -> None:
        self._node.get_logger().info(f"Callback 2 received message {msg} and will now sleep for 5 seconds")
        time.sleep(5)
        self._node.get_logger().info(f"Callback 2 is done sleeping after receiving {msg}")


@ros_process.main()
def main() -> None:
    _ = Listener()
    ros_process.wait_for_shutdown()


if __name__ == "__main__":
    main()
