#!/usr/bin/env python

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Initialize ROS node
rospy.init_node('camera_calibration_node')

# Initialize the bridge between ROS and OpenCV
bridge = CvBridge()

# Define callback function to process camera images
def image_callback(msg):
    try:
        # Convert ROS image message to OpenCV format
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")

        # Perform checkerboard detection (similar to your original script)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Define the checkerboard size (number of inner corners)
        width = 9  # Adjust as needed
        height = 6  # Adjust as needed

        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, (width, height), None)

        if ret:
            # Draw and display the corners (for visualization)
            cv2.drawChessboardCorners(frame, (width, height), corners, ret)
            cv2.imshow("Image", frame)
            cv2.waitKey(1)

            # Object points (3D coordinates of checkerboard corners)
            objp = np.zeros((width * height, 3), np.float32)
            objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)
            objp *= square_size  # Adjust square_size as needed

            # Image points (2D coordinates of detected corners)
            imgpoints.append(corners.reshape(-1, 2))
            objpoints.append(objp)

    except Exception as e:
        rospy.logerr(e)

# Subscribe to the ZED camera topic
camera_topic = "/zed/rgb/image_raw"  # Replace with the correct topic name
rospy.Subscriber(camera_topic, Image, image_callback)

# Initialize object and image points lists
objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane

# Define the size of each square on the checkerboard (in meters)
square_size = 0.024  # Adjust as needed

# Define the calibration process
def calibrate_camera():
    global objpoints, imgpoints

    # Perform the camera calibration
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, (2560,1800), None, None)

    # Print and save the calibration parameters
    print("Camera Matrix (mtx):")
    print(mtx)

    print("Distortion Coefficients (dist):")
    print(dist)

    # Save the calibration parameters to a file
    np.save("calibration_matrix", mtx)
    np.save("distortion_coefficients", dist)

    # Close any OpenCV windows
    cv2.destroyAllWindows()

    # Stop the ROS node
    rospy.signal_shutdown("Calibration completed.")

if __name__ == '__main__':
    # Execute the camera calibration process
    calibrate_camera()

    # Spin ROS node
    rospy.spin()