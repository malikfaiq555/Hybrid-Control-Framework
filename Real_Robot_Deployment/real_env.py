#!/usr/bin/env python3

"""
real_env.py: Minimal real-world environment for DRL navigation on TurtleBot3 + ROS.
- Reads odometry and 2D lidar/pointcloud.
- Publishes velocity.
- Gets goal from /pos_rot topic (e.g., VR interface or RViz).
- Provides state, and checks for goal reached.
"""

import math
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from unity_robotics_demo_msgs.msg import PosRot
from tf.transformations import euler_from_quaternion
from math import atan2

GOAL_REACHED_DIST = 0.1
TIME_DELTA = 0.1  # seconds

class Env:
    def __init__(self, environment_dim):
        self.environment_dim = environment_dim
        self.odom_x = 0
        self.odom_y = 0
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.velodyne_data = np.ones(self.environment_dim) * 10
        self.last_odom_x = 0.0
        self.last_odom_y = 0.0
        self.last_odom_theta = 0.0
        self.digi_twin_x = 0.0
        self.digi_twin_y = 0.0

        self.gaps = [[-np.pi / 2 - 0.03, -np.pi / 2 + np.pi / self.environment_dim]]
        for m in range(self.environment_dim - 1):
            self.gaps.append([self.gaps[m][1], self.gaps[m][1] + np.pi / self.environment_dim])
        self.gaps[-1][-1] += 0.03

        rospy.init_node("TD3_real", anonymous=True)
        self.digi_twin_pose = rospy.Subscriber("/pos_rot", PosRot, self.newPosRot)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.velodyne = rospy.Subscriber("/2D_Point_Cloud", PointCloud2, self.point_cloud_callback, queue_size=1)
        self.odom = rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=1)

    def newPosRot(self, msg):
        self.digi_twin_x = msg.pos_z // 3
        self.digi_twin_y = msg.pos_x

    def point_cloud_callback(self, v):
        data = list(pc2.read_points(v, skip_nans=False, field_names=("x", "y", "z")))
        self.velodyne_data = np.ones(self.environment_dim) * 10
        for pt in data:
            if pt[0] == 0 or pt[1] == 0:
                continue
            dot = pt[0]
            mag1 = math.hypot(pt[0], pt[1])
            beta = math.acos(dot / mag1) * np.sign(pt[1])
            dist = math.hypot(pt[0], pt[1])
            for j, (low, high) in enumerate(self.gaps):
                if low <= beta < high:
                    self.velodyne_data[j] = min(self.velodyne_data[j], dist)
                    break

    def odom_callback(self, od_data):
        self.last_odom_x = od_data.pose.pose.position.x
        self.last_odom_y = od_data.pose.pose.position.y
        rot_q = od_data.pose.pose.orientation
        (_, _, self.last_odom_theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def step(self, action):
        vel_cmd = Twist()
        vel_cmd.linear.x = action[0]
        vel_cmd.angular.z = action[1]
        self.vel_pub.publish(vel_cmd)
        rospy.sleep(TIME_DELTA)

        self.odom_x = self.last_odom_x
        self.odom_y = self.last_odom_y
        angle = self.last_odom_theta
        distance = np.linalg.norm([self.odom_x - self.goal_x, self.odom_y - self.goal_y])
        skew_x = self.goal_x - self.odom_x
        skew_y = self.goal_y - self.odom_y
        theta = atan2(skew_y, skew_x) - angle
        theta = (theta + np.pi) % (2 * np.pi) - np.pi  # Normalize to [-pi, pi]

        target = distance < GOAL_REACHED_DIST
        done = target

        robot_state = [distance, theta, action[0], action[1]]
        state = np.append([self.velodyne_data], robot_state)
        return state, done, target

    def reset(self):
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.change_goal()
        v_state = [self.velodyne_data]
        distance = np.linalg.norm([self.odom_x - self.goal_x, self.odom_y - self.goal_y])
        skew_x = self.goal_x - self.odom_x
        skew_y = self.goal_y - self.odom_y
        theta = atan2(skew_y, skew_x)
        theta = (theta + np.pi) % (2 * np.pi) - np.pi
        robot_state = [distance, theta, 0.0, 0.0]
        state = np.append(v_state, robot_state)
        return state

    def reached_goal(self):
        vel_cmd = Twist()
        vel_cmd.linear.x = 0.0
        vel_cmd.angular.z = 0.0
        self.vel_pub.publish(vel_cmd)

    def change_goal(self):
        self.goal_x = self.digi_twin_x
        self.goal_y = self.digi_twin_y
