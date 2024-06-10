# Copyright (c) 2024 Boston Dynamics AI Institute Inc.  All rights reserved.

from datetime import datetime, timedelta
from typing import Union

from rclpy.duration import Duration
from rclpy.time import Time


def as_proper_time(time: Union[int, float, datetime, Time]) -> Time:
    """Return `time` as a proper Time object."""
    if isinstance(time, int):
        return Time(nanoseconds=time)
    if isinstance(time, float):
        return Time(seconds=time)
    if isinstance(time, datetime):
        return Time(seconds=time.timestamp())
    return time


def as_proper_duration(duration: Union[int, float, timedelta, Duration]) -> Duration:
    """Return `duration` as a proper Duration object."""
    if isinstance(duration, int):
        return Duration(nanoseconds=duration)
    if isinstance(duration, float):
        return Duration(seconds=duration)
    if isinstance(duration, timedelta):
        return Duration(seconds=duration.total_seconds())
    return duration
