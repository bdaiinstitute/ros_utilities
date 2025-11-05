# Copyright (c) 2023 Robotics and AI Institute LLC dba RAI Institute.  All rights reserved.
import threading
from typing import Optional

from rclpy.clock import Clock
from rclpy.context import Context
from rclpy.utilities import get_default_context

from synchros2.clock import wait_for


def wait_for_shutdown(
    *,
    timeout_sec: Optional[float] = None,
    clock: Optional[Clock] = None,
    context: Optional[Context] = None,
) -> bool:
    """Wait for context shutdown.

    Args:
        timeout_sec: optional timeout for wait, wait indefinitely by default.
        clock: optional clock to use for timeout waits, waits on system clock by default.
        context: context to wait on, use default context by default.

    Returns:
        True if shutdown, False on timeout.
    """
    if context is None:
        context = get_default_context()
    event = threading.Event()
    context.on_shutdown(event.set)
    return wait_for(event, clock=clock, timeout_sec=timeout_sec)
