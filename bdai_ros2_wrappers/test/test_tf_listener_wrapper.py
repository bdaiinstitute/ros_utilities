import asyncio
import time

from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
import tf2_ros

import pytest

from bdai_ros.wrappers.tf_listener_wrapper import TFListenerWrapper

ROBOT = "test_robot"
CAMERA = "camera_1"
FRAME_ID = f"{ROBOT}/body"
CHILD_FRAME_ID = f"{ROBOT}/{CAMERA}"


class MockTFPublisher(Node):

    def __init__(self, frame_id, child_frame_id):
        super().__init__("mock_tf_publisher")

        self.frame_id = frame_id
        self.child_frame_id = child_frame_id

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def publish_transform(
        self,
        translation,
        rotation,
        timestamp,
    ):
        t = TransformStamped()

        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id

        t.transform = Transform(
            translation=Vector3(x=translation[0], y=translation[1], z=translation[2]),
            rotation=Quaternion(w=rotation[0], x=rotation[1], y=rotation[2], z=rotation[3]),
        )

        self.tf_broadcaster.sendTransform(t)

    async def publish_transform_async(
        self,
        translation=[0, 0, 0],
        rotation=[1, 0, 0, 0],
        timestamp=None,
        delay=0,
    ):
        if delay > 0:
            await asyncio.sleep(delay)

        self.publish_transform(translation, rotation, timestamp)


def check_tf_lookup(tf_listener, translation, rotation, timestamp, timeout=0, wait_for_frames=False):
    try:
        t = tf_listener.lookup_a_tform_b(
            FRAME_ID,
            CHILD_FRAME_ID,
            timestamp,
            timeout=timeout,
            wait_for_frames=wait_for_frames,
        )

        assert t.position.x == translation[0]
        assert t.position.y == translation[1]
        assert t.position.z == translation[2]

        assert t.rotation.w == rotation[0]
        assert t.rotation.x == rotation[1]
        assert t.rotation.y == rotation[2]
        assert t.rotation.z == rotation[3]

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as err:
        raise (err)


# I do not know if this is the best way to test asynchronous routines. We may want to do this with
# multiple nodes and service calls to synchronize the message passing.
async def delayed_lookup(tf_publisher,
                         tf_listener,
                         translation,
                         rotation,
                         delay_sec=0,
                         timeout=0,
                         wait_for_frames=False,
                         check_init=False):
    timestamp = tf_publisher.get_clock().now()
    timestamp += Duration(seconds=delay_sec)
    loop = asyncio.get_event_loop()
    delay_task = loop.create_task(tf_publisher.publish_transform_async(translation, rotation, timestamp, delay_sec))

    await asyncio.sleep(0)
    try:
        if not check_init:
            await loop.run_in_executor(
                None, lambda: check_tf_lookup(tf_listener, translation, rotation, timestamp, timeout, wait_for_frames))
        else:
            await loop.run_in_executor(None, lambda: tf_listener.wait_for_init(FRAME_ID, CHILD_FRAME_ID))
    finally:
        # I do not know if this is necessary
        await delay_task


# Using the pytest feature allows us to clean up the listener so this doesn't hang at the end
@pytest.fixture
def tf_listener():
    args = ["test", "--ros-args", "-p", f"robot:={ROBOT}"]
    rclpy.init(args=args)

    _tf_listener = TFListenerWrapper("mock_tf_wrapper")
    yield _tf_listener
    # We have to do this to make sure the test exits cleanly even if it fails
    _tf_listener.shutdown()
    rclpy.shutdown()


def test_tf_listener_wrapper(tf_listener):
    """
    Unit tests for the TFListenerWrapper class
    """

    tf_publisher = MockTFPublisher(FRAME_ID, CHILD_FRAME_ID)

    rclpy.spin_once(tf_publisher, timeout_sec=0.01)

    ## Non-existent transform throws LookupException
    print()
    print("Looking up nonexisting transform")
    timestamp = tf_publisher.get_clock().now()
    lookup_err = None
    try:
        trans = tf_listener.lookup_a_tform_b(FRAME_ID, CHILD_FRAME_ID, timestamp)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as err:
        lookup_err = err
    assert isinstance(lookup_err, tf2_ros.LookupException)
    print(f"LookupException Thrown")

    ## A non-existent transform with a timeout but without wait_for_frames returns a LookupException immediately
    # (Time-based tests can be flaky but in this case we're trying to tell the difference between 20s and immediately
    #  so hopefully we'll be ok.)
    print()
    print("Looking up nonexisting transform with a timeout but without wait_for_frames")
    timestamp = tf_publisher.get_clock().now()
    lookup_err = None
    start = time.time()
    try:
        trans = tf_listener.lookup_a_tform_b(FRAME_ID, CHILD_FRAME_ID, timestamp, timeout=20.0)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as err:
        lookup_err = err
    end = time.time()
    assert isinstance(lookup_err, tf2_ros.LookupException)
    print(f"LookupException Thrown")
    assert (end - start < 10.0)

    ## A non-existent transform with a timeout but with wait_for_frames properly waits
    print()
    print("Waiting for a non-existent transform that eventually comes in")
    asyncio.run(
        delayed_lookup(tf_publisher,
                       tf_listener, [4.0, 5.0, 6.0], [0.0, 1.0, 0.0, 0.0],
                       delay_sec=2,
                       timeout=5.0,
                       wait_for_frames=True))

    ## Existing transform returns correctly
    print()
    print("Looking up existing transform")
    tf_publisher.publish_transform([1.0, 2.0, 3.0], [1.0, 0.0, 0.0, 0.0], timestamp)

    rclpy.spin_once(tf_publisher, timeout_sec=0.01)

    trans = tf_listener.lookup_a_tform_b(FRAME_ID, CHILD_FRAME_ID, timestamp)
    assert trans.position.x == 1.0
    assert trans.position.y == 2.0
    assert trans.position.z == 3.0

    assert trans.rotation.w == 1.0
    assert trans.rotation.x == 0.0
    assert trans.rotation.y == 0.0
    assert trans.rotation.z == 0.0
    print(f"Correct Transform Found")

    ## A future transform request with no timeout returns ExtrapolationException
    print()
    print("Looking up future transform without wait")
    rclpy.spin_once(tf_publisher, timeout_sec=0.01)

    lookup_err = None
    try:
        asyncio.run(
            delayed_lookup(tf_publisher, tf_listener, [4.0, 5.0, 6.0], [0.0, 1.0, 0.0, 0.0], delay_sec=2, timeout=0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as err:
        lookup_err = err
    assert isinstance(lookup_err, tf2_ros.ExtrapolationException)
    print(f"ExtrapolationException Thrown")

    ## A future transform request with an insufficent timeout returns ExtrapolationException
    print()
    print("Looking up future transform with insufficient wait")
    rclpy.spin_once(tf_publisher, timeout_sec=0.01)

    lookup_err = None
    try:
        asyncio.run(
            delayed_lookup(tf_publisher, tf_listener, [7.0, 8.0, 9.0], [0.0, 0.0, 1.0, 0.0], delay_sec=2, timeout=1))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as err:
        lookup_err = err
    assert isinstance(lookup_err, tf2_ros.ExtrapolationException)
    print(f"ExtrapolationException Thrown")

    ## A future transform request with sufficient timeout returns correctly
    print()
    print("Looking up future transform with sufficient wait")
    rclpy.spin_once(tf_publisher, timeout_sec=0.01)

    asyncio.run(
        delayed_lookup(tf_publisher, tf_listener, [10.0, 11.0, 12.0], [0.0, 0.0, 0.0, 1.0], delay_sec=2, timeout=4))
    print(f"Correct Transform Found")

    print("TFListenerWrapper Tests Complete")


def test_tf_listener_wrapper_wait(tf_listener):
    tf_publisher = MockTFPublisher(FRAME_ID, CHILD_FRAME_ID)

    rclpy.spin_once(tf_publisher, timeout_sec=0.01)

    asyncio.run(
        delayed_lookup(tf_publisher, tf_listener, [4.0, 5.0, 6.0], [0.0, 1.0, 0.0, 0.0], delay_sec=2, check_init=True))

    print("Successfully initialized TF")


if __name__ == "__main__":
    test_tf_listener_wrapper()
