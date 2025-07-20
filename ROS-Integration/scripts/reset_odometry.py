#!/usr/bin/env python3
"""
ROS Node: reset_odometry

Description:
    Utility node to reset the odometry of TurtleBot3.
    Publishes to the `/reset` topic (std_msgs/Empty), which is supported by
    TurtleBot3's `turtlebot3_core` and some simulation nodes for zeroing odometry.

    Waits for /odom data, monitors the yaw (orientation), and attempts up to five resets.
    Each reset is followed by a short wait (for hardware/sim to zero odom).

Usage:
    $ rosrun <your_package> reset_odometry.py
Topics:
    Subscribes: /odom   (nav_msgs/Odometry)
    Publishes:  /reset  (std_msgs/Empty)
References:
    - [TurtleBot3 Odometry Reset](http://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#odometry-reset)
"""

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from tf.transformations import euler_from_quaternion
from math import pi
import numpy as np

# State tracking
count = 0
start = rospy.get_rostime()
data = np.zeros(7)  # [move, turn, drift, yaw, x, y, seconds]

def QtoYaw(orientation_q):
    """
    Convert quaternion to yaw (heading).
    """
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (_, _, yaw) = euler_from_quaternion(orientation_list)
    if yaw < 0:
        yaw = 2 * pi + yaw    # Normalize to 0...2*pi for convenience
    return yaw

def get_odom(msg):
    """
    Odometry callback: update global odom state.
    """
    global start, data
    yaw = QtoYaw(msg.pose.pose.orientation)
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    move = msg.twist.twist.linear.x
    drift = msg.twist.twist.linear.y
    turn = msg.twist.twist.angular.z
    now = msg.header.stamp - start
    seconds = now.to_sec()
    data = np.copy((move, turn, drift, yaw, x, y, seconds))

def main():
    global count, data
    rospy.init_node('reset_odometry')

    # Subscribers and publishers
    subO = rospy.Subscriber('/odom', Odometry, get_odom)
    reset_odom = rospy.Publisher('/reset', Empty, queue_size=1)

    rospy.loginfo("Waiting for odom callback to start...")
    while data[6] == 0 and not rospy.is_shutdown():  # Wait for first odom message
        rospy.sleep(0.1)

    while count < 5 and not rospy.is_shutdown():
        datacopy = np.copy(data)
        yaw = datacopy[3]
        if yaw > pi:
            yaw = yaw - 2 * pi  # Normalize to -pi..pi

        print("At %.3f secs, x %.4f , y %.4f : yaw %.4f, drift %.6f" %
              (datacopy[6], datacopy[4], datacopy[5], yaw, datacopy[2]))

        # Check if yaw is close to zero (reset successful)
        if abs(yaw) < 0.005:
            rospy.loginfo("Yaw zeroed.")
            # Optionally, break here if only need first zeroing:
            # break

        # Only publish reset for first 4 iterations
        if count < 4:
            reset_odom.publish(Empty())
            rospy.loginfo("Odometry reset request sent.")
            rospy.sleep(4.0)  # Allow time for reset to take effect

        count += 1

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
