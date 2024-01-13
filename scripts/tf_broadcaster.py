#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

def tf_broadcaster():
    rospy.init_node('tf_broadcaster_node')
    tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

    # Define the static transform from "world" to "zed_depth_camera"
    static_transform_stamped = geometry_msgs.msg.TransformStamped()
    static_transform_stamped.header.stamp = rospy.Time.now()
    static_transform_stamped.header.frame_id = "map"  # Parent frame
    static_transform_stamped.child_frame_id = "zed_depth_camera"  # Child frame

    # Set the translation (position) of the "zed_depth_camera" frame
    static_transform_stamped.transform.translation.x = 0.0  # Adjust as needed
    static_transform_stamped.transform.translation.y = 0.0  # Adjust as needed
    static_transform_stamped.transform.translation.z = 5.0  # Adjust as needed

    quaternion = quaternion_from_euler(0, 3.14159265, 0)  # 180 degrees in radians

    # Set the rotation (orientation) of the "zed_depth_camera" frame
    static_transform_stamped.transform.rotation.x = quaternion[0]
    static_transform_stamped.transform.rotation.y = quaternion[1]
    static_transform_stamped.transform.rotation.z = quaternion[2]
    static_transform_stamped.transform.rotation.w = quaternion[3]

    # Publish the static transform
    tf_broadcaster.sendTransform(static_transform_stamped)

    rospy.spin()

if __name__ == '__main__':
    try:
        tf_broadcaster()
    except rospy.ROSInterruptException:
        pass
