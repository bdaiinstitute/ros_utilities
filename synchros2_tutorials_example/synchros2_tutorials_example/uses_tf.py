import synchros2.process as ros_process
import synchros2.scope as ros_scope


@ros_process.main(uses_tf=True)
def main() -> None:
    """Main"""
    node = ros_scope.node()
    # The process-wide TF listener is available everywhere as ros_scope.tf_listener()
    tf_listener = ros_scope.tf_listener()
    base_frame = "world"
    target_frame = "robot"
    # Wait for our transform to become available
    node.get_logger().info(f"Waiting for transform from {base_frame} to {target_frame} to be available")
    tf_listener.wait_for_a_tform_b(base_frame, target_frame)
    world_t_robot = tf_listener.lookup_a_tform_b(base_frame, target_frame)
    node.get_logger().info(f"Transform from world to robot is {world_t_robot}")


if __name__ == "__main__":
    main()
