# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.
import logging
import threading
from typing import List

from rcl_interfaces.msg import Log
from rclpy.clock import ROSClock
from rclpy.time import Time

from bdai_ros2_wrappers.futures import unwrap_future
from bdai_ros2_wrappers.logging import LoggingSeverity, as_memoizing_logger, logs_to_ros
from bdai_ros2_wrappers.scope import ROSAwareScope
from bdai_ros2_wrappers.subscription import Subscription


def test_memoizing_logger(verbose_ros: ROSAwareScope) -> None:
    messages: List[str] = []
    cv = threading.Condition()

    def callback(message: Log) -> None:
        nonlocal messages, cv
        with cv:
            messages.append(message.msg)
            cv.notify()

    assert verbose_ros.node is not None
    verbose_ros.node.create_subscription(Log, "/rosout", callback, 10)

    logger = as_memoizing_logger(verbose_ros.node.get_logger())
    logger.set_level(LoggingSeverity.INFO)

    assert not logger.debug("Debug message should not be logged")

    for i in range(2):
        is_first_iteration = i == 0
        did_log_once = logger.error(
            f"Error message should have been logged only if {i} == 0",
            once=True,
        )
        assert did_log_once == is_first_iteration
        did_skip_first_log = logger.error(
            f"Error message should have been logged only if {i} != 0",
            skip_first=True,
        )
        assert did_skip_first_log != is_first_iteration

    assert logger.warning("Warning message always logged", once=True)
    assert logger.warning("Warning message always logged", once=True)

    fake_clock = ROSClock()
    fake_clock._set_ros_time_is_active(True)

    num_throttled_logs = 2
    num_attempts_per_sec = 5
    for i in range(num_attempts_per_sec * num_throttled_logs):
        fake_clock.set_ros_time_override(Time(seconds=float(i) / num_attempts_per_sec))
        assert logger.info(
            "Info message should be throttled",
            throttle_duration_sec=1.0,
            throttle_time_source_type=fake_clock,
        ) == (i % num_attempts_per_sec == 0)

    expected_messages = [
        "Error message should have been logged only if 0 == 0",
        "Error message should have been logged only if 1 != 0",
        "Warning message always logged",
        "Warning message always logged",
    ] + ["Info message should be throttled"] * num_throttled_logs

    def all_messages_arrived() -> bool:
        return len(messages) == len(expected_messages)

    with cv:
        assert cv.wait_for(all_messages_arrived, timeout=5.0)
    assert messages == expected_messages


def test_log_forwarding(verbose_ros: ROSAwareScope) -> None:
    assert verbose_ros.node is not None
    rosout = Subscription(Log, "/rosout", 10, node=verbose_ros.node)
    assert unwrap_future(rosout.publisher_matches(1), timeout_sec=5.0) > 0

    with logs_to_ros(verbose_ros.node):
        logger = logging.getLogger("my_logger")
        logger.setLevel(logging.INFO)
        logger.propagate = True  # ensure propagation is enabled
        logger.info("test")

    log = unwrap_future(rosout.update, timeout_sec=5.0)
    # NOTE(hidmic) why are log levels of bytestring type !?
    assert log.level == int.from_bytes(Log.INFO, byteorder="little")
    assert log.name == verbose_ros.node.get_logger().name
    assert log.msg == "(logging.my_logger) test"
