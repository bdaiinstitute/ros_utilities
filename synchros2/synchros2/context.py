# Copyright (c) 2023 Boston Dynamics AI Institute LLC.  All rights reserved.
import threading
from typing import Optional

from rclpy.context import Context
from rclpy.utilities import get_default_context


def wait_for_shutdown(*, timeout_sec: Optional[float] = None, context: Optional[Context] = None) -> bool:
    """Wait for context shutdown.

    Args:
        timeout_sec: optional timeout for wait, wait indefinitely by default.
        context: context to wait on, use default context by default.

    Returns:
        True if shutdown, False on timeout.
    """
    if context is None:
        context = get_default_context()
    event = threading.Event()
    context.on_shutdown(event.set)
    return event.wait(timeout_sec)
