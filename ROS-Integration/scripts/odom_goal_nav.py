#!/usr/bin/env python3
"""
ROS Node: speed_controller
Description: Moves a mobile robot toward a specified goal position. 
             First aligns robot orientation, then moves forward.
Topics:
    Subscribes: /odom (nav_msgs/Odometry)
    Publishes:  /cmd_vel (geometry_msgs/Twist)
"""

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt

# Global variables to hold robot's current position and orientation
x = 0.0
y = 0.0
theta = 0.0

def odom_callback(msg):
    """
    Callback function to handle incoming Odometry messages.
    Updates global x, y, and theta (orientation).
    """
    global x, y, theta

    # Extract position
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    # Extract orientation in quaternion form and convert to Euler angles
    orientation_q = msg.pose.pose.orientation
    (_, _, theta) = euler_from_quaternion([
        orientation_q.x,
        orientation_q.y,
        orientation_q.z,
        orientation_q.w
    ])

    rospy.loginfo(f"Odometry updated: x={x:.2f}, y={y:.2f}, theta={theta:.2f}")

def move_to_goal():
    """
    Main control loop that navigates the robot towards the goal.
    """
    rospy.init_node("speed_controller")

    # Subscriber to odometry data
    rospy.Subscriber("/odom", Odometry, odom_callback)

    # Publisher to send velocity commands
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    speed = Twist()
    rate = rospy.Rate(10)  # 10 Hz control rate for smoother movement

    # Define goal position (you can modify as required)
    goal = Point(x=5.0, y=5.0, z=0.0)
    goal_threshold = 0.2  # Acceptable radius around goal

    while not rospy.is_shutdown():
        # Calculate distance and angle to goal
        distance_to_goal = sqrt((goal.x - x)**2 + (goal.y - y)**2)
        angle_to_goal = atan2(goal.y - y, goal.x - x)

        rospy.loginfo(f"Distance to goal: {distance_to_goal:.2f}")

        # Check if goal is reached within a tolerance
        if distance_to_goal < goal_threshold:
            rospy.loginfo("Goal reached! Stopping robot.")
            speed.linear.x = 0.0
            speed.angular.z = 0.0
            pub.publish(speed)
            break  # Exit loop after reaching goal

        # Rotate towards goal if angular difference is significant
        angle_diff = angle_to_goal - theta
        if abs(angle_diff) > 0.1:
            rospy.loginfo(f"Rotating: angle_diff={angle_diff:.2f}")
            speed.linear.x = 0.0
            speed.angular.z = 0.3 if angle_diff > 0 else -0.3
        else:
            rospy.loginfo("Aligned with goal direction, moving forward.")
            speed.linear.x = 0.2  # Slower speed for safety
            speed.angular.z = 0.0

        pub.publish(speed)
        rate.sleep()

if __name__ == "__main__":
    try:
        move_to_goal()
    except rospy.ROSInterruptException:
        pass
