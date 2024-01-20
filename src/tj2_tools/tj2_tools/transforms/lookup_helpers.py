from typing import Optional, Union

import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped


def lookup_transform(
    tf_buffer: tf2_ros.Buffer,
    parent_link: str,
    child_link: str,
    time_window: Optional[Union[rospy.Time, rospy.Duration]] = None,
    timeout: Optional[rospy.Duration] = None,
    silent: bool = False,
) -> Optional[TransformStamped]:
    """
    Call tf_buffer.lookup_transform. Return None if the look up fails
    """
    if time_window is None:
        time_window = rospy.Time(0)
    else:
        time_window = rospy.Time.now() - time_window

    if timeout is None:
        timeout = rospy.Duration(1.0)  # type: ignore

    try:
        return tf_buffer.lookup_transform(parent_link, child_link, time_window, timeout)
    except (
        tf2_ros.LookupException,  # type: ignore
        tf2_ros.ConnectivityException,  # type: ignore
        tf2_ros.ExtrapolationException,  # type: ignore
        tf2_ros.InvalidArgumentException,  # type: ignore
    ) as e:
        if not silent:
            rospy.logwarn("Failed to look up %s to %s. %s" % (parent_link, child_link, e))
        return None


def transform_pose(
    tf_buffer: tf2_ros.Buffer,
    pose_stamped: PoseStamped,
    destination_frame: str,
    time_window: Optional[Union[rospy.Time, rospy.Duration]] = None,
    timeout: Optional[rospy.Duration] = None,
    silent: bool = False,
) -> Optional[PoseStamped]:
    transform = lookup_transform(
        tf_buffer,
        destination_frame,
        pose_stamped.header.frame_id,
        time_window,
        timeout,
        silent,
    )
    if transform is None:
        return None
    return tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
