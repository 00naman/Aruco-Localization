import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import math
import argparse  # Import the argparse module for command-line arguments

# Initialize variables to store the robot's current position and orientation
robot_x = 0.0
robot_y = 0.0
yaw = 0.0
position_threshold = 0.05

def odometry_callback(msg):
    global robot_x, robot_y, yaw

    # Extract the robot's position from the Odometry message
    robot_x = msg.pose.pose.position.x
    robot_y = msg.pose.pose.position.y

    # Extract the robot's orientation (yaw) from the Odometry message
    orientation_quat = msg.pose.pose.orientation
    orientation_list = [orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w]
    _, _, yaw = euler_from_quaternion(orientation_list)

def move_to_target(target_x, target_y):
    global robot_x, robot_y, yaw

    # Create a ROS node for the movement control
    rospy.init_node('move_to_target_node')

    # Subscribe to the /aruco/odometry topic
    rospy.Subscriber('/aruco/odometry', Odometry, odometry_callback)

    # Create a publisher to send velocity commands to the robot
    cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Set the rate at which to send control commands
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Calculate the distance to the target
        distance_to_target = math.sqrt((target_x - robot_x)**2 + (target_y - robot_y)**2)

        # Check if the robot has reached the target within the threshold
        if distance_to_target < position_threshold:
            rospy.loginfo("Target reached!")
            # Stop the robot
            cmd_vel_msg = Twist()
            cmd_vel_publisher.publish(cmd_vel_msg)
            break

        # Calculate the angle to the target (yaw angle)
        angle_to_target = math.atan2(target_y - robot_y, target_x - robot_x)

        # Calculate the angular velocity needed to align with the target
        angular_velocity = 0.5 * (angle_to_target - yaw)

        # Create a Twist message to control the robot's velocity
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.2  # Adjust the linear velocity as needed
        cmd_vel_msg.angular.z = angular_velocity  # Set the angular velocity

        # Publish the velocity command
        cmd_vel_publisher.publish(cmd_vel_msg)

        rate.sleep()

if __name__ == '__main__':
    # Parse command-line arguments for target_x and target_y
    parser = argparse.ArgumentParser(description='Move the robot to a target position.')
    parser.add_argument('target_x', type=float, help='X-coordinate of the target position')
    parser.add_argument('target_y', type=float, help='Y-coordinate of the target position')
    args = parser.parse_args()

    # Set the target position from command-line arguments
    target_x = args.target_x
    target_y = args.target_y

    try:
        move_to_target(target_x, target_y)
    except rospy.ROSInterruptException:
        pass
