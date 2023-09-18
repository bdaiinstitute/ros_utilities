# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.
from typing import Optional
from datetime import datetime

from rosbag2_py import Recorder, RecordOptions, StorageOptions
from bdai.utilities.path import get_bdai_path
import os
from rclpy.node import Node
import threading
import rclpy


ROS_BAG_NAME_TIMESTAMP_FORMAT: str = "rosbag2_%Y_%m_%d-%H_%M_%S_%f"

class ROSBagRecorder(object):
    def __init__(
        self,
        bag_file_name: str,
        bag_file_dir: Optional[str] = None,
        storage_id: str = "sqlite3",
    ) -> None:
        self._rosbag_recorder = Recorder()

        self._record_options = RecordOptions()
        self._record_options.all = True

        bag_file_name += "_"

        self._node_name = bag_file_name + "recorder"

        dt = datetime.now().strftime(ROS_BAG_NAME_TIMESTAMP_FORMAT)
        bag_name = "{}{}".format(bag_file_name, dt)
        if bag_file_dir:
            self._bag_path = os.path.join(bag_file_dir, bag_name)
        else:
            self._bag_path = os.path.join(get_bdai_path(), "recordings", bag_name)

        self._storage_options = StorageOptions(
            uri=self._bag_path,
            storage_id=storage_id)

    @property
    def bag_path(self) -> str:
        return self._bag_path

    def start_recording(self) -> None:
        if rclpy.ok():
            self._rosbag_recorder.record(self._storage_options, self._record_options)

    def stop_recording(self) -> None:
        self._rosbag_recorder.cancel()
