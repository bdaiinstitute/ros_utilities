# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.
from threading import Event
from typing import Optional

from rclpy.context import Context
from rclpy.task import Future
from rclpy.utilities import get_default_context


def wait_for_future(
    future: Future,
    timeout_sec: Optional[float] = None,
    *,
    context: Optional[Context] = None
) -> bool:
    """Blocks while waiting for a future to become done

    Args:
        future (Future): The future to be waited on
        timeout_sec (Optional[float]): An optional timeout for how long to wait
        context (Optional[Context]): Current context (will use the default if none is given)

    Returns:
        bool: True if successful, False if the timeout was triggered
    """
    if context is None:
        context = get_default_context()
    event = Event()
    context.on_shutdown(event.set)
    future.add_done_callback(lambda _: event.set())
    event.wait(timeout=timeout_sec)
    return future.done()
