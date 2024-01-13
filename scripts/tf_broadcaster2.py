#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg

def publish_map_to_odom_transform():
    rospy.init_node('map_to_odom_tf_publisher')
    tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

    static_transform_stamped = geometry_msgs.msg.TransformStamped()
    static_transform_stamped.header.stamp = rospy.Time.now()
    static_transform_stamped.header.frame_id = "map"  # Parent frame (your "map" frame)
    static_transform_stamped.child_frame_id = "odom"  # Child frame (your "odom" frame)

    # Set the translation (position) of the "odom" frame relative to "map"
    static_transform_stamped.transform.translation.x = 0.0  # Adjust as needed
    static_transform_stamped.transform.translation.y = 0.0  # Adjust as needed
    static_transform_stamped.transform.translation.z = 0.0  # Adjust as needed

    # Set the rotation (orientation) of the "odom" frame relative to "map"
    static_transform_stamped.transform.rotation.x = 0.0
    static_transform_stamped.transform.rotation.y = 0.0
    static_transform_stamped.transform.rotation.z = 0.0
    static_transform_stamped.transform.rotation.w = 1.0  # Identity quaternion

    # Publish the static transform
    rate = rospy.Rate(10.0)  # Publish rate (adjust as needed)
    while not rospy.is_shutdown():
        tf_broadcaster.sendTransform(static_transform_stamped)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_map_to_odom_transform()
    except rospy.ROSInterruptException:
        pass
