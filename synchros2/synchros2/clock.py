# Copyright (c) 2025 Boston Dynamics AI Institute LLC.  All rights reserved.

import threading
from typing import Optional

from rclpy.clock import Clock, ClockChange, ClockType, JumpThreshold, TimeJump
from rclpy.duration import Duration


def wait_for(
    event: threading.Event,
    *,
    clock: Optional[Clock] = None,
    timeout_sec: Optional[float] = None,
) -> bool:
    """Wait for event to be set.

    A threading.Event.wait() equivalent that is clock aware.

    Args:
        event: event to wait on.
        clock: optional clock to use for timeout waits, waits on system clock by default.
        timeout_sec: optional timeout for wait, wait indefinitely by default.

    Returns:
        True if event is set, False on timeout.
    """
    if timeout_sec is None:
        return event.wait()

    if clock is None:
        clock = Clock()

    if clock.clock_type != ClockType.ROS_TIME:
        return event.wait(timeout_sec)

    if not event.is_set():
        timeout_event = threading.Event()
        deadline = clock.now() + Duration(seconds=timeout_sec)

        def on_time_jump(time_jump: TimeJump) -> None:
            assert clock is not None
            time_source_changed = (
                time_jump.clock_change == ClockChange.ROS_TIME_ACTIVATED
                or time_jump.clock_change == ClockChange.ROS_TIME_DEACTIVATED
            )
            timed_out = clock.now() >= deadline
            if event.is_set() or time_source_changed or timed_out:
                timeout_event.set()

        threshold = JumpThreshold(
            min_forward=Duration(nanoseconds=1),
            min_backward=None,
            on_clock_change=True,
        )
        with clock.create_jump_callback(threshold, post_callback=on_time_jump):
            timeout_event.wait()

    return event.is_set()
