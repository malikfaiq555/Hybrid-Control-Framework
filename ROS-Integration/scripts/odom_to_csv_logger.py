#!/usr/bin/env python3
"""
ROS Node: odom_to_csv_logger

Description:
    Subscribes to /odom (nav_msgs/Odometry) and logs position, orientation (yaw), 
    and velocity data to a CSV file for later plotting/analysis.

    This is helpful for visualizing trajectories, analyzing robot performance,
    or debugging experiments.

Usage:
    $ rosrun <your_package> odom_to_csv_logger.py
    (You can also launch with roslaunch if you prefer.)

    CSV file will be saved as 'robot_trajectory_log.csv' in the working directory.

Topics:
    Subscribes: /odom (nav_msgs/Odometry)
"""

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import pandas as pd

# Lists for logging data
positions_x = []
positions_y = []
orientations_yaw = []
linear_vel_x = []
angular_vel_z = []

def odom_callback(msg):
    """
    Callback function to process odometry messages and store relevant data.
    """
    # Position
    px = msg.pose.pose.position.x
    py = msg.pose.pose.position.y

    # Orientation (yaw)
    orientation_q = msg.pose.pose.orientation
    (_, _, yaw) = euler_from_quaternion([
        orientation_q.x,
        orientation_q.y,
        orientation_q.z,
        orientation_q.w
    ])

    # Velocities
    lx = msg.twist.twist.linear.x
    az = msg.twist.twist.angular.z

    # Append to lists (rounded for cleaner CSV)
    positions_x.append(round(px, 4))
    positions_y.append(round(py, 4))
    orientations_yaw.append(round(yaw, 4))
    linear_vel_x.append(round(lx, 4))
    angular_vel_z.append(round(az, 4))

def main():
    rospy.init_node("odom_to_csv_logger")

    # Subscribe to odometry
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rate = rospy.Rate(1)  # Log to CSV every 1 second (adjust as needed)
    csv_filename = "robot_trajectory_log.csv"

    rospy.loginfo("Logging robot odometry to %s. Ctrl+C to stop.", csv_filename)

    while not rospy.is_shutdown():
        # DataFrame built from currently logged data
        df = pd.DataFrame({
            'position_x': positions_x,
            'position_y': positions_y,
            'euler_yaw': orientations_yaw,
            'linearV_x': linear_vel_x,
            'angularV_z': angular_vel_z
        })
        df.to_csv(csv_filename, index=False, encoding='utf-8')
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
