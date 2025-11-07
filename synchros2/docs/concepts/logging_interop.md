# Logging Interoperability

`synchros2` includes functionality to bridge Python standard logging with the ROS 2 logging system. This allows you to use Python's logging APIs and have logs forwarded to ROS 2, including `/rosout`.

## Log bridging

Managing multiple, independent logging systems is impractical. So is reworking a codebase to accommodate either. To avoid both scenarios, use `synchros2.logging.logs_to_ros` explicitly:

```python
import logging
import rclpy
from synchros2.logging import logs_to_ros

rclpy.init()
node = rclpy.create_node("my_node")
with logs_to_ros(node):
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.INFO)
    logger.info("Logged using standard logging")
rclpy.shutdown()
```

Or implicitly through process-wide APIs:

```python
import logging
from synchros2.process as ros_process

@ros_process.main()
def main():
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.INFO)
    logger.info("Logged using standard logging")

if __name__ == "__main__":
    main()
```

`logging` logs will propagate up the logger hierarchy and then be forwarded to the logger of the corresponding node (the one provided in the first case, or the main prebaked node in the second case) and thus at least printed to console and published to `/rosout`. Make sure severity levels are set appropriately for logs to get through.

## API review

You can then use standard Python logging:

```python
import logging
logging.basicConfig()
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
logger.info("Logged using standard logging")
```

Or use standalone `rclpy` logging:

```python
import rclpy.logging
logger = rclpy.logging.get_logger(__name__)
logger.set_level(rclpy.logging.LoggingSeverity.INFO)
logger.info("Logged using a standalone rclpy logger")
```

Or node-based logging:

```python
import rclpy
import rclpy.logging
node = rclpy.create_node("my_node")
logger = node.get_logger()
logger.set_level(rclpy.logging.LoggingSeverity.INFO)
logger.info("Logged using an rclpy node logger")
```

All of these will be handled by the ROS 2 logging system, and node-based logs will also be published to `/rosout`.
