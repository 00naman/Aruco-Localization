import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import math
from tf.transformations import quaternion_from_matrix
from scipy.spatial.transform import Rotation as R

# Manually define camera calibration parameters (example values)
fx = fy = (3840 / 2) / math.tan(1.92 / 2)  # Focal length in y-direction
cx = 3840 / 2
cy = 2160 / 2  # Principal point in y-direction
k1 = 0.0  # Distortion coefficient k1
k2 = 0.0  # Distortion coefficient k2
p1 = 0.0  # Distortion coefficient p1
p2 = 0.0  # Distortion coefficient p2
k3 = 0.0  # Distortion coefficient k3

def aruco_detection_callback(data):
    try:
        # Convert the ROS image message to an OpenCV image
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

        # Convert the image to grayscale for ArUco marker detection
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Define the ArUco dictionary and parameters
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        parameters = cv2.aruco.DetectorParameters_create()

        # Detect ArUco markers
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            for i in range(len(ids)):
                # Estimate the marker pose in 3D space
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.5, camera_matrix, distortion_coeff)
                rmat, _ = cv2.Rodrigues(rvec)
                # print(rmat)
                # Convert the rotation matrix to a quaternion
                rotation = R.from_matrix(rmat)
                quaternion = rotation.as_quat()
                if tvec is not None:
                    # Access the elements of tvec and cast them to float
                    tvec_x = round(float(tvec[0][0][0]), 3)
                    tvec_y = round(float(tvec[0][0][1]), 3)
                    tvec_z = round(float(tvec[0][0][2]), 3)

                    # Create an Odometry message and populate it with data
                    odometry_msg = Odometry()
                    odometry_msg.header.stamp = rospy.Time.now()
                    odometry_msg.header.frame_id = "map"
                    odometry_msg.child_frame_id = "zed_depth_camera"
                    odometry_msg.pose.pose.position.x = tvec_x
                    odometry_msg.pose.pose.position.y = tvec_y
                    odometry_msg.pose.pose.position.z = tvec_z


                    # You can calculate and set the orientation if you have the necessary data
                    #quaternion_custom = [quaternion[3], quaternion[0], quaternion[1], quaternion[2]]
                    # odometry_msg.pose.pose.orientation = ...
                    odometry_msg.pose.pose.orientation.x = quaternion[0]
                    odometry_msg.pose.pose.orientation.y = quaternion[1]
                    odometry_msg.pose.pose.orientation.z = quaternion[2]
                    odometry_msg.pose.pose.orientation.w = quaternion[3]

                    # Publish the Odometry message
                    odometry_publisher.publish(odometry_msg)

                    

                    rospy.loginfo(f"ArUco marker {ids[i]} found at position (x, y, z): ({tvec_x}, {tvec_y}, {tvec_z})")
        else:
            rospy.loginfo("ArUco marker not found.")

    except CvBridgeError as e:
        print(e)

if __name__ == '__main__':
    rospy.init_node('aruco_detection_node')
    bridge = CvBridge()

    # Manually define camera matrix and distortion coefficients
    camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    distortion_coeff = np.array([k1, k2, p1, p2, k3])

    # Create a publisher for the /aruco/odometry topic
    odometry_publisher = rospy.Publisher('/aruco/odometry', Odometry, queue_size=10)

    # Subscribe to the image topic (adjust the topic name as needed)
    image_topic = "/zed/rgb/image_raw"
    rospy.Subscriber(image_topic, Image, aruco_detection_callback)

    # Set the update rate (3 seconds)
    rate = rospy.Rate(0.999)

    while not rospy.is_shutdown():
        # Publish messages every 3 seconds
        rate.sleep()