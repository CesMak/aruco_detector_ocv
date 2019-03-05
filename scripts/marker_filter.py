#!/usr/bin/env python

import numpy as np
import rospy
import geometry_msgs.msg
import tf2_ros
from tf.transformations import quaternion_slerp


def translation_to_numpy(t):
    return np.array([t.x, t.y, t.z])


def quaternion_to_numpy(q):
    return np.array([q.x, q.y, q.z, q.w])


if __name__ == '__main__':
    rospy.init_node('marker_filter')
    alpha = rospy.get_param('~alpha', 0.9)
    parent_frame_id = rospy.get_param('~parent_frame_id', 'kinect2_link')
    marker_id = rospy.get_param('~marker_id', 'marker_id0')
    marker_filtered_id = rospy.get_param(
        '~marker_filtered_id', 'marker_id0_filtered')
    rate_value = rospy.get_param('~rate_value', 125)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    br = tf2_ros.TransformBroadcaster()

    marker_pose = None
    marker_pose0 = None
    rate = rospy.Rate(rate_value)
    while not rospy.is_shutdown():
        marker_pose0 = marker_pose
        # Lookup the transform
        try:
            marker_pose_new = tfBuffer.lookup_transform(
                parent_frame_id, marker_id, rospy.Time())

            if not marker_pose_new is None:
                marker_pose = marker_pose_new
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(e)

        if marker_pose is None:
            rate.sleep()
            continue

        # Apply running average filter to translation and rotation
        if not marker_pose0 is None:
            rotation0 = quaternion_to_numpy(marker_pose0.transform.rotation)
            rotation = quaternion_to_numpy(marker_pose.transform.rotation)
            rotation_interpolated = quaternion_slerp(
                rotation0, rotation, 1 - alpha)

            translation0 = translation_to_numpy(
                marker_pose0.transform.translation)
            translation = translation_to_numpy(
                marker_pose.transform.translation)
            translation = alpha * translation0 + (1 - alpha) * translation

            # Update pose of the marker
            marker_pose.transform.rotation.x = rotation_interpolated[0]
            marker_pose.transform.rotation.y = rotation_interpolated[1]
            marker_pose.transform.rotation.z = rotation_interpolated[2]
            marker_pose.transform.rotation.w = rotation_interpolated[3]
            marker_pose.transform.translation.x = translation[0]
            marker_pose.transform.translation.y = translation[1]
            marker_pose.transform.translation.z = translation[2]

        # Create new transform and broadcast it
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent_frame_id
        t.child_frame_id = marker_filtered_id
        t.transform = marker_pose.transform
        br.sendTransform(t)
        rate.sleep()
