# Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC.  All rights reserved.

import contextlib
import functools
import inspect
import logging
import typing
import warnings

from rclpy.clock import Clock
from rclpy.duration import Duration

# NOTE(hidmic): tinkering with implementation details is not ideal,
# but there is not a lot of room for alternatives. This inconvenience
# will go away when and if this module is contributed upstream.
from rclpy.impl.implementation_singleton import rclpy_implementation as impl
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.logging import LoggingSeverity
from rclpy.node import Node

from synchros2.utilities import cap, skip, throttle

SEVERITY_MAP = {
    logging.NOTSET: LoggingSeverity.UNSET,
    logging.DEBUG: LoggingSeverity.DEBUG,
    logging.INFO: LoggingSeverity.INFO,
    logging.WARN: LoggingSeverity.WARN,
    logging.ERROR: LoggingSeverity.ERROR,
    logging.CRITICAL: LoggingSeverity.FATAL,
}
REVERSE_SEVERITY_MAP = {v: k for k, v in SEVERITY_MAP.items()}


if not typing.TYPE_CHECKING:
    import os.path
    import xml.etree.ElementTree as ET

    import ament_index_python
    import packaging.version

    share_directory = ament_index_python.get_package_share_directory("rclpy")
    tree = ET.parse(os.path.join(share_directory, "package.xml"))
    version = tree.getroot().find("version")
    if packaging.version.parse(version.text) >= packaging.version.parse("3.9.0"):
        warnings.warn("TODO: use subloggers in RcutilsLogHandler implementation", stacklevel=1)


@functools.lru_cache(maxsize=1024)
def make_logging_function(
    name: str,
    level: LoggingSeverity,
    origin: inspect.Traceback,
    throttle_duration_sec: typing.Optional[float] = None,
    throttle_time_source: typing.Optional[Clock] = None,
    skip_first: typing.Optional[bool] = None,
    once: typing.Optional[bool] = None,
) -> typing.Callable[[str], bool]:
    """Make a fast rclpy logging function.

    A logging function takes a log message and returns True if the message was logged, False otherwise.
    A message will not be logged if:

    * the logger is not enabled for the logging function's severity level (i.e. it is less than
      the severity level of the logger), or
    * some logging filter (throttling, once, skip) causes the message to be skipped.

    Logging filters will only be evaluated if the logger is enabled for the logging function's severity level.

    Args:
        name: logger name.
        level: log severity level.
        origin: log call site.
        throttle_duration_sec: optional period for throttling, in seconds.
        throttle_time_source: optional time source for throttling, defaults to system time.
        skip_first: if True, skip the first log.
        once: if True, only log once.

    Returns:
        a logging function.
    """

    def log(message: str) -> bool:
        impl.rclpy_logging_rcutils_log(
            level,
            name,
            message,
            origin.function,
            origin.filename,
            origin.lineno,
        )
        return True

    if throttle_duration_sec is not None:
        throttle_period = Duration(
            seconds=throttle_duration_sec,
        )
        log = throttle(
            log,
            min_period=throttle_period,
            time_source=throttle_time_source,
            fill_value=False,
        )
    if skip_first:
        log = skip(log, num_times=1, fill_value=False)
    if once:
        log = cap(log, num_times=1, fill_value=False)
    return log


class MemoizingRcutilsLogger:
    """An alternative, more efficient implementation of RcutilsLogger.

    MemoizingRcutilsLogger caches logging call configuration to speed up
    subsequent invocations.
    """

    def __init__(self, name: str = "") -> None:
        """Initializes the logger with the given `name`, or the root logger if none is provided."""
        self.name = name

    def get_child(self, name: str) -> "MemoizingRcutilsLogger":
        """Gets a child logger with the given `name`."""
        if not name:
            raise ValueError("Need name for child logger.")
        if self.name:
            name = self.name + "." + name
        return MemoizingRcutilsLogger(name)

    def set_level(self, level: typing.Union[int, LoggingSeverity]) -> None:
        """Sets logger severity `level`."""
        impl.rclpy_logging_set_logger_level(self.name, LoggingSeverity(level))

    def get_effective_level(self) -> LoggingSeverity:
        """Gets the effective logger severity level.

        The effective severity level of a logger is the first severity level set in the logger
        genealogy (ie. its own or that of its parent or that of its grandparent and so on), or
        the default severity level when no severity level is.
        """
        return LoggingSeverity(impl.rclpy_logging_get_logger_effective_level(self.name))

    def is_enabled_for(self, level: typing.Union[int, LoggingSeverity]) -> bool:
        """Checks whether the logger is enabled for logs at the given severity `level`."""
        return impl.rclpy_logging_logger_is_enabled_for(self.name, LoggingSeverity(level))

    def log(
        self,
        message: str,
        level: typing.Union[int, LoggingSeverity],
        origin: typing.Optional[inspect.Traceback] = None,
        throttle_duration_sec: typing.Optional[float] = None,
        throttle_time_source_type: typing.Optional[Clock] = None,
        skip_first: typing.Optional[bool] = None,
        once: typing.Optional[bool] = None,
    ) -> bool:
        """Log a message with the specified severity `level`.

        A message will not be logged if:

        * the logger is not enabled for the logging function's severity level (i.e. it is less than
          the severity level of the logger), or
        * some logging filter (throttling, once, skip) causes the message to be skipped.

        Logging filters will only be evaluated if the logger is enabled for the logging function's severity level.

        Args:
            message: message to be logged.
            level: severity level of the message.
            origin: optional log call site, defaults to the caller one level up the call stack.
            throttle_duration_sec: optional period for throttling, in seconds.
            throttle_time_source_type: optional time source for throttling, defaults to system time.
            skip_first: if True, skip the first log.
            once: if True, only log once.

        Returns:
            whether the message was logged or not.
        """
        if not self.is_enabled_for(level):
            return False
        if origin is None:
            current_frame = inspect.currentframe()
            assert current_frame is not None
            outer_frame = current_frame.f_back
            assert outer_frame is not None
            origin = inspect.getframeinfo(outer_frame, context=0)
        do_log = make_logging_function(
            self.name,
            LoggingSeverity(level),
            origin,
            throttle_duration_sec,
            throttle_time_source_type,
            skip_first,
            once,
        )
        return do_log(message)

    def debug(self, message: str, origin: typing.Optional[inspect.Traceback] = None, **kwargs: typing.Any) -> bool:
        """Log a message with `DEBUG` severity via :py:method:MemoizingRcutilsLogger.log:."""
        if origin is None:
            current_frame = inspect.currentframe()
            assert current_frame is not None
            outer_frame = current_frame.f_back
            assert outer_frame is not None
            origin = inspect.getframeinfo(outer_frame, context=0)
        return self.log(message, LoggingSeverity.DEBUG, origin, **kwargs)

    def info(self, message: str, origin: typing.Optional[inspect.Traceback] = None, **kwargs: typing.Any) -> bool:
        """Log a message with `INFO` severity via :py:method:MemoizingRcutilsLogger.log:."""
        if origin is None:
            current_frame = inspect.currentframe()
            assert current_frame is not None
            outer_frame = current_frame.f_back
            assert outer_frame is not None
            origin = inspect.getframeinfo(outer_frame, context=0)
        return self.log(message, LoggingSeverity.INFO, origin, **kwargs)

    def warning(self, message: str, origin: typing.Optional[inspect.Traceback] = None, **kwargs: typing.Any) -> bool:
        """Log a message with `WARN` severity via :py:method:MemoizingRcutilsLogger.log:."""
        if origin is None:
            current_frame = inspect.currentframe()
            assert current_frame is not None
            outer_frame = current_frame.f_back
            assert outer_frame is not None
            origin = inspect.getframeinfo(outer_frame, context=0)
        return self.log(message, LoggingSeverity.WARN, origin, **kwargs)

    def warn(self, message: str, origin: typing.Optional[inspect.Traceback] = None, **kwargs: typing.Any) -> bool:
        """Log a message with `WARN` severity via :py:method:MemoizingRcutilsLogger.log:.

        Deprecated in favor of :py:classmethod:RcutilsLogger.warning:.
        """
        warnings.warn(
            "MemoizingRcutilsLogger.warn() is deprecated in favor of MemoizingRcutilsLogger.warning()",
            DeprecationWarning,
            stacklevel=2,
        )
        if origin is None:
            current_frame = inspect.currentframe()
            assert current_frame is not None
            outer_frame = current_frame.f_back
            assert outer_frame is not None
            origin = inspect.getframeinfo(outer_frame, context=0)
        return self.warning(message, origin, **kwargs)

    def error(self, message: str, origin: typing.Optional[inspect.Traceback] = None, **kwargs: typing.Any) -> bool:
        """Log a message with `ERROR` severity via :py:method:MemoizingRcutilsLogger.log:."""
        if origin is None:
            current_frame = inspect.currentframe()
            assert current_frame is not None
            outer_frame = current_frame.f_back
            assert outer_frame is not None
            origin = inspect.getframeinfo(outer_frame, context=0)
        return self.log(message, LoggingSeverity.ERROR, origin, **kwargs)

    def fatal(self, message: str, origin: typing.Optional[inspect.Traceback] = None, **kwargs: typing.Any) -> bool:
        """Log a message with `FATAL` severity via :py:method:MemoizingRcutilsLogger.log:."""
        if origin is None:
            current_frame = inspect.currentframe()
            assert current_frame is not None
            outer_frame = current_frame.f_back
            assert outer_frame is not None
            origin = inspect.getframeinfo(outer_frame, context=0)
        return self.log(message, LoggingSeverity.FATAL, origin, **kwargs)


def as_memoizing_logger(logger: RcutilsLogger) -> MemoizingRcutilsLogger:
    """Turns a regular `rclpy` logger into a memoizing one."""
    return MemoizingRcutilsLogger(logger.name)


class RcutilsLogHandler(logging.Handler):
    """A `logging.Handler` subclass to forward log records to the ROS 2 logging system."""

    default_formatter = logging.Formatter("(logging.%(name)s) %(message)s")

    def __init__(self, node: Node, level: typing.Union[int, str] = logging.NOTSET) -> None:
        """Constructor"""
        super().__init__(level=level)
        self.node = node
        self.logger = node.get_logger()
        if self.level != logging.NOTSET:
            self.logger.set_level(SEVERITY_MAP[self.level])
        self.setFormatter(self.default_formatter)

    def setLevel(self, level: typing.Union[int, str]) -> None:
        """Sets the threshold for this handler to level.

        Logging messages which are less severe than level will be ignored. When a handler is created, the level is set
        to NOTSET (which causes all messages to be processed).
        """
        super().setLevel(level)
        self.logger.set_level(SEVERITY_MAP[self.level])

    def emit(self, record: logging.LogRecord) -> None:
        """Do whatever it takes to actually log the specified logging record."""
        try:
            message = self.format(record)
            severity = SEVERITY_MAP[record.levelno]
            # NOTE(hidmic): this bypasses the rclpy logger API to avoid the extra meaningless
            # computations (e.g. call stack inspection).
            impl.rclpy_logging_rcutils_log(
                severity,
                self.logger.name,
                message,
                record.funcName,
                record.pathname,
                record.lineno,
            )
        except Exception:
            self.handleError(record)


@contextlib.contextmanager
def logs_to_ros(node: Node) -> typing.Iterator[None]:
    """Forwards root `logging.Logger` logs to the ROS 2 logging system.

    Note that logs are subject to severity level thresholds and propagation
    semantics at both the `logging` module and the ROS 2 logging system. For
    instance, for an informational log made from a non-root `logging` logger
    to find its way to a ROS 2 logging system sink like the `/rosout` topic:

    - the entire `logging` hierarchy all the way up to the root logger, and the
      ROS 2 node logger must be configured with a severity level at or below
      INFO;
    - and the `logging` hierarchy must be configured to propagate logs (true by default).

    Args:
        node: a ROS 2 node, necessary for rosout logging (if enabled).
    """
    root = logging.getLogger()
    root.setLevel(REVERSE_SEVERITY_MAP[node.get_logger().get_effective_level()])
    handler = RcutilsLogHandler(node)
    root.addHandler(handler)
    try:
        yield
    finally:
        root.removeHandler(handler)
