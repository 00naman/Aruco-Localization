import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import math
import argparse

# Define the desired target position (x, y) in the robot's frame
target_x = 2.0  # Adjust the desired x-coordinate as needed
target_y = 2.0  # Adjust the desired y-coordinate as needed

# Define PID controller parameters for linear motion control
Kp_linear = 0.5  # Proportional gain for linear motion
Ki_linear = 0.0  # Integral gain for linear motion (set to 0 for now)
Kd_linear = 0.1  # Derivative gain for linear motion

# Define PID controller parameters for angular motion control
Kp_angular = 0.5 # Proportional gain for angular motion
Ki_angular = 0.0  # Integral gain for angular motion (set to 0 for now)
Kd_angular = 0.1  # Derivative gain for angular motion

# Define maximum linear and angular velocities
max_linear_velocity = 0.4  # Maximum linear velocity in m/s
max_angular_velocity = 1.0  # Maximum angular velocity in rad/s

# Initialize variables to store the robot's current position and orientation
robot_x = 0.0
robot_y = 0.0
yaw = 0.0

# Initialize variables for PID control
linear_integral = 0.0
linear_previous_error = 0.0
angular_integral = 0.0
angular_previous_error = 0.0

def odometry_callback(msg):
    global robot_x, robot_y, yaw

    # Extract the robot's position from the Odometry message
    robot_x = msg.pose.pose.position.x
    robot_y = msg.pose.pose.position.y

    # Extract the robot's orientation (yaw) from the Odometry message
    orientation_quat = msg.pose.pose.orientation
    orientation_list = [orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w]
    _, _, yaw = euler_from_quaternion(orientation_list)

def pid_control(target_x, target_y):
    global robot_x, robot_y, yaw, linear_integral, linear_previous_error, angular_integral, angular_previous_error

    # Create a ROS node for the PID control
    rospy.init_node('pid_control_node')

    # Subscribe to the /aruco/odometry topic
    rospy.Subscriber('/aruco/odometry', Odometry, odometry_callback)

    # Create a publisher to send velocity commands to the robot
    cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Set the rate at which to send control commands
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Calculate the distance to the target
        distance_to_target = math.sqrt((target_x - robot_x)**2 + (target_y - robot_y)**2)

        # Calculate the angle to the target (yaw angle)
        angle_to_target = math.atan2(target_y - robot_y, target_x - robot_x)

        # Calculate the error for linear motion control
        linear_error = distance_to_target

        # Update the integral term for linear motion control
        linear_integral += linear_error

        # Calculate the derivative term for linear motion control
        linear_derivative = linear_error - linear_previous_error

        # Calculate the control command for linear motion using PID
        linear_control_command = Kp_linear * linear_error + Ki_linear * linear_integral + Kd_linear * linear_derivative

        # Limit the linear velocity to the maximum value
        linear_control_command = min(max_linear_velocity, max(-max_linear_velocity, linear_control_command))

        # Update the previous error for linear motion control
        linear_previous_error = linear_error

        # Calculate the error for angular motion control (orientation adjustment)
        angular_error = angle_to_target - yaw

        # Ensure the angular error is within the range [-pi, pi]
        if angular_error > math.pi:
            angular_error -= 2 * math.pi
        elif angular_error < -math.pi:
            angular_error += 2 * math.pi

        # Update the integral term for angular motion control
        angular_integral += angular_error

        # Calculate the derivative term for angular motion control
        angular_derivative = angular_error - angular_previous_error

        # Calculate the control command for angular motion using PID
        angular_control_command = Kp_angular * angular_error + Ki_angular * angular_integral + Kd_angular * angular_derivative

        # Limit the angular velocity to the maximum value
        angular_control_command = min(max_angular_velocity, max(-max_angular_velocity, angular_control_command))

        # Update the previous error for angular motion control
        angular_previous_error = angular_error

        # Create a Twist message to control the robot's velocity
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_control_command  # Linear velocity control
        cmd_vel_msg.angular.z = angular_control_command  # Angular velocity control

        # Publish the velocity command
        cmd_vel_publisher.publish(cmd_vel_msg)

        # Check if the robot has reached the target within the threshold
        if distance_to_target < position_threshold:
            rospy.loginfo("Target reached!")
            # Stop the robot
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.0
            cmd_vel_publisher.publish(cmd_vel_msg)
            break

        rate.sleep()

if __name__ == '__main__':
    try:
        position_threshold = 0.05  # Define a position threshold for reaching the target
        parser = argparse.ArgumentParser(description='Move the robot to a target position.')
        parser.add_argument('target_x', type=float, help='X-coordinate of the target position')
        parser.add_argument('target_y', type=float, help='Y-coordinate of the target position')
        args = parser.parse_args()

        # Set the target position from command-line arguments
        target_x = args.target_x
        target_y = args.target_y

        pid_control(target_x, target_y)
    except rospy.ROSInterruptException:
        pass
