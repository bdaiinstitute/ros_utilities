import synchros2.process as ros_process
import synchros2.scope as ros_scope
from synchros2.tf_listener_wrapper import TFListenerWrapper


@ros_process.main()
def main() -> None:
    """Main"""
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
