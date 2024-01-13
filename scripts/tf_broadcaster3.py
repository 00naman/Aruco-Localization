#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
import math

def main():
    # Initialize ROS node
    rospy.init_node('tf_publisher_node')

    # Create a TF broadcaster
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    rate = rospy.Rate(10)  # Publish rate (adjust as needed)

    while not rospy.is_shutdown():
        # Define the transform between base frame and ArUco link frame
        transform = TransformStamped()

        # Header
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "link_chassis"  # Replace with your robot's base frame
        transform.child_frame_id = "aruco_link"   # Replace with the ArUco marker link frame

        # Translation
        transform.transform.translation.x = 0.0   # Adjust X translation as needed
        transform.transform.translation.y = 0.0   # No Y translation
        transform.transform.translation.z = 0.0   # No Z translation

        # Rotation (Quaternion)
        yaw_angle = math.pi / 2.0  # 90-degree yaw rotation
        quat_w = math.cos(yaw_angle / 2.0)
        quat_z = math.sin(yaw_angle / 2.0)
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = quat_z
        transform.transform.rotation.w = quat_w

        # Publish the transform
        tf_broadcaster.sendTransform(transform)

        rate.sleep()

if __name__ == '__main__':
    main()
