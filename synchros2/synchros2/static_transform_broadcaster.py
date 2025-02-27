# Copyright (c) 2023 Boston Dynamics AI Institute LLC.  All rights reserved.
from typing import Dict, Iterable, Optional, Union, cast

from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from tf2_msgs.msg import TFMessage


class StaticTransformBroadcaster:
    """A modified :class:`tf2_ros.StaticTransformBroadcaster` that stores transforms sent through it.

    This matches ``rclcpp::StaticTransformBroadcaster`` behavior.
    """

    DEFAULT_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)

    def __init__(self, node: Node, qos: Optional[Union[QoSProfile, int]] = None) -> None:
        """Constructor.

        :param node: The ROS2 node.
        :param qos: A QoSProfile or a history depth to apply to the publisher.
        """
        if qos is None:
            qos = StaticTransformBroadcaster.DEFAULT_QOS
        self._net_transforms: Dict[str, TransformStamped] = {}
        self._pub = node.create_publisher(TFMessage, "/tf_static", qos)

    def sendTransform(self, transform: Union[TransformStamped, Iterable[TransformStamped]]) -> None:
        """Send a transform, or a list of transforms, to the Buffer associated with this TransformBroadcaster.

        Args:
            transform: A transform or list of transforms to send.
        """
        try:
            transforms = list(transform)
        except TypeError:  # in which case, transform is not iterable
            transforms = [cast(TransformStamped, transform)]
        self._net_transforms.update({tf.child_frame_id: tf for tf in transforms})
        self._pub.publish(TFMessage(transforms=list(self._net_transforms.values())))
