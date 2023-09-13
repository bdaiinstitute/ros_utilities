# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.

import contextlib
import logging
import typing

import rclpy.logging

# NOTE(hidmic): tinkering with implementation details is not ideal,
# but there is not a lot of room for alternatives. This inconvenience
# will go away when and if this module is contributed upstream.
from rclpy.impl.implementation_singleton import rclpy_implementation as impl
from rclpy.node import Node

SEVERITY_MAP = {
    logging.NOTSET: rclpy.logging.LoggingSeverity.UNSET,
    logging.DEBUG: rclpy.logging.LoggingSeverity.DEBUG,
    logging.INFO: rclpy.logging.LoggingSeverity.INFO,
    logging.WARN: rclpy.logging.LoggingSeverity.WARN,
    logging.ERROR: rclpy.logging.LoggingSeverity.ERROR,
    logging.CRITICAL: rclpy.logging.LoggingSeverity.FATAL,
}


class RcutilsLogHandler(logging.Handler):
    """A `logging.Handler` subclass to forward log records to the ROS 2 logging system."""

    default_formatter = logging.Formatter("(logging.%(name)s) %(message)s")

    def __init__(self, node: Node, level: typing.Union[int, str] = logging.NOTSET) -> None:
        super().__init__(level=level)
        self.node = node
        self.logger = node.get_logger()
        if self.level != logging.NOTSET:
            self.logger.set_level(SEVERITY_MAP[self.level])
        self.setFormatter(self.default_formatter)

    def setLevel(self, level: typing.Union[int, str]) -> None:
        super().setLevel(level)
        self.logger.set_level(SEVERITY_MAP[self.level])

    def emit(self, record: logging.LogRecord) -> None:
        try:
            message = self.format(record)
            severity = SEVERITY_MAP[record.levelno]
            # NOTE(hidmic): this bypasses the rclpy logger API
            # to avoid the extra meaningless computations (e.g.
            # call stack inspection).
            #
            # TODO(hidmic): use subloggers when migrating to Iron
            # (or the feature is backported to Humble).
            impl.rclpy_logging_rcutils_log(
                severity, self.logger.name, message, record.funcName, record.pathname, record.lineno
            )
        except Exception:
            self.handleError(record)


@contextlib.contextmanager
def logs_to_ros(node: Node) -> typing.Iterator[None]:
    """
    Forwards root `logging.Logger` logs to the ROS 2 logging system.

    :param node: a ROS 2 node, necessary for rosout logging (if enabled).
    """
    root = logging.getLogger()
    handler = RcutilsLogHandler(node)
    root.addHandler(handler)
    try:
        yield
    finally:
        root.removeHandler(handler)
