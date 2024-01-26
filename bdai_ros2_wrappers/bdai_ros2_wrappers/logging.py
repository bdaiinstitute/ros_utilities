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


if not typing.TYPE_CHECKING:
    import os.path
    import warnings
    import xml.etree.ElementTree as ET

    import ament_index_python
    import packaging.version

    share_directory = ament_index_python.get_package_share_directory("rclpy")
    tree = ET.parse(os.path.join(share_directory, "package.xml"))
    version = tree.getroot().find("version")
    if packaging.version.parse(version.text) >= packaging.version.parse("3.9.0"):
        warnings.warn("TODO: use subloggers in RcutilsLogHandler implementation", stacklevel=1)


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
    handler = RcutilsLogHandler(node)
    root.addHandler(handler)
    try:
        yield
    finally:
        root.removeHandler(handler)
