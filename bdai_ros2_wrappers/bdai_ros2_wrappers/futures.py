# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.
from threading import Event
from typing import Any, Optional

from rclpy.task import Future


def wait_for_future(future: Future, timeout_sec: Optional[float] = None) -> bool:
    """Blocks while waiting for a future to become done

    Args:
        future (Future): The future to be waited on
        timeout_sec (Optional[float]): An optional timeout for how long to wait

    Returns:
        bool: True if successful, False if the timeout was triggered
    """
    event = Event()

    def done_callback(_: Any) -> None:
        nonlocal event
        event.set()

    future.add_done_callback(done_callback)
    return event.wait(timeout=timeout_sec)
