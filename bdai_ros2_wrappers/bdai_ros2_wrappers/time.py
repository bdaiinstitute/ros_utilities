# Copyright (c) 2024 Boston Dynamics AI Institute Inc.  All rights reserved.

from datetime import datetime, timedelta
from typing import Union

from rclpy.duration import Duration
from rclpy.time import Time


def as_proper_time(time: Union[int, float, datetime, Time]) -> Time:
    """Return `time` as a proper Time object.

    Note that for scalar times the convention is that floating point times
    are expressed in seconds since some clock epoch, and integral times are
    expressed in nanoseconds since some clock epoch.
    """
    if isinstance(time, int):
        return Time(nanoseconds=time)
    if isinstance(time, float):
        return Time(seconds=time)
    if isinstance(time, datetime):
        return Time(seconds=time.timestamp())
    return time


def as_proper_duration(duration: Union[int, float, timedelta, Duration]) -> Duration:
    """Return `duration` as a proper Duration object.

    Note that for scalar durations the convention is that floating point durations
    are expressed in seconds, and integral durations are expressed in nanoseconds.
    """
    if isinstance(duration, int):
        return Duration(nanoseconds=duration)
    if isinstance(duration, float):
        return Duration(seconds=duration)
    if isinstance(duration, timedelta):
        return Duration(seconds=duration.total_seconds())
    return duration
