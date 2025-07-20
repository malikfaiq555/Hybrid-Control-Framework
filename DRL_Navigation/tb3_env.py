"""
Generic TD3 RL Environment for ROS+Gazebo

- Accepts any launch file, state vector size, and lidar/pointcloud topic.
- Designed for use with DRL training/testing in simulation.
- Adapt check_pos and sensor topics to your scenario/robot as needed.
"""

import math
import os
import random
import subprocess
import time
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from squaternion import Quaternion

GOAL_REACHED_DIST = 0.3
COLLISION_DIST = 0.35
TIME_DELTA = 0.1

def check_pos(x, y):
    """
    Placeholder for obstacle filtering.
    User should modify based on their world/map!
    """
    goal_ok = True
    # Example: simple workspace boundary (square)
    if x > 4.5 or x < -4.5 or y > 4.5 or y < -4.5:
        goal_ok = False
    return goal_ok

class TD3Env:
    def __init__(self, launchfile="abc.launch", environment_dim=24, pointcloud_topic="/2D_Point_Cloud"):
        self.environment_dim = environment_dim
        self.lidar_data = np.ones(self.environment_dim) * 10
        self.last_odom = None

        self.goal_x = 0.0
        self.goal_y = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0

        # Gaps for lidar projection (evenly slice 180 degrees)
        self.gaps = [[-np.pi / 2 - 0.03, -np.pi / 2 + np.pi / self.environment_dim]]
        for m in range(self.environment_dim - 1):
            self.gaps.append(
                [self.gaps[m][1], self.gaps[m][1] + np.pi / self.environment_dim]
            )
        self.gaps[-1][-1] += 0.03

        # --- ROS & Gazebo startup ---
        try:
            subprocess.Popen(["roscore"])
            time.sleep(2)
        except Exception:
            print("roscore already running or failed to launch.")

        rospy.init_node("td3_env", anonymous=True)

        # Start Gazebo with user-specified launch file (if not already running)
        if not launchfile.startswith("/"):
            fullpath = os.path.join(os.getcwd(), launchfile)
        else:
            fullpath = launchfile
        if not os.path.exists(fullpath):
            raise IOError(f"Launch file {fullpath} does not exist.")
        try:
            subprocess.Popen(["roslaunch", fullpath])
            print(f"Gazebo launched with {launchfile}")
            time.sleep(5)
        except Exception as e:
            print("roslaunch already running or failed to launch:", e)

        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.pc_sub = rospy.Subscriber(pointcloud_topic, PointCloud2, self.pointcloud_callback)

    def pointcloud_callback(self, msg):
        data = list(pc2.read_points(msg, skip_nans=False, field_names=("x", "y", "z")))
        self.lidar_data = np.ones(self.environment_dim) * 10
        for i in range(len(data)):
            x, y = data[i][0], data[i][1]
            if x == 0 or y == 0:
                continue
            dot = x * 1 + y * 0
            mag1 = math.sqrt(x ** 2 + y ** 2)
            mag2 = 1.0
            beta = math.acos(dot / (mag1 * mag2)) * np.sign(y)
            dist = math.sqrt(x ** 2 + y ** 2)
            for j in range(len(self.gaps)):
                if self.gaps[j][0] <= beta < self.gaps[j][1]:
                    self.lidar_data[j] = min(self.lidar_data[j], dist)
                    break

    def odom_callback(self, msg):
        self.last_odom = msg
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

    def step(self, action):
        vel_cmd = Twist()
        vel_cmd.linear.x = float(action[0])
        vel_cmd.angular.z = float(action[1])
        self.vel_pub.publish(vel_cmd)
        time.sleep(TIME_DELTA)

        done, collision, min_lidar = self.observe_collision(self.lidar_data)

        if self.last_odom is None:
            rospy.logwarn("No odom received yet.")
            return np.zeros(self.environment_dim + 4), 0.0, True, False

        dx = self.goal_x - self.odom_x
        dy = self.goal_y - self.odom_y
        distance = np.linalg.norm([dx, dy])

        orientation = self.last_odom.pose.pose.orientation
        quaternion = Quaternion(
            orientation.w,
            orientation.x,
            orientation.y,
            orientation.z,
        )
        yaw = quaternion.to_euler(degrees=False)[2]
        angle_to_goal = math.atan2(dy, dx) - yaw
        angle_to_goal = (angle_to_goal + np.pi) % (2 * np.pi) - np.pi

        linear_vel = self.last_odom.twist.twist.linear.x
        angular_vel = self.last_odom.twist.twist.angular.z

        target = False
        if distance < GOAL_REACHED_DIST:
            target = True
            done = True

        robot_state = [distance, angle_to_goal, linear_vel, angular_vel]
        state = np.concatenate([self.lidar_data, robot_state])
        reward = self.get_reward(target, collision, action, min_lidar)
        return state, reward, done, target

    def reset(self):
        # Wait for odometry data
        while self.last_odom is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        self.odom_x = self.last_odom.pose.pose.position.x
        self.odom_y = self.last_odom.pose.pose.position.y
        # Place a random goal (user: adjust bounds/check_pos as needed)
        while True:
            self.goal_x = self.odom_x + random.uniform(-3, 3)
            self.goal_y = self.odom_y + random.uniform(-3, 3)
            if check_pos(self.goal_x, self.goal_y):
                break
        time.sleep(TIME_DELTA)
        state = np.concatenate([self.lidar_data, [self.goal_dist(), self.goal_angle(), 0.0, 0.0]])
        return state

    def goal_dist(self):
        return np.linalg.norm([self.odom_x - self.goal_x, self.odom_y - self.goal_y])

    def goal_angle(self):
        dx = self.goal_x - self.odom_x
        dy = self.goal_y - self.odom_y
        yaw = 0.0
        if self.last_odom:
            orientation = self.last_odom.pose.pose.orientation
            quaternion = Quaternion(
                orientation.w,
                orientation.x,
                orientation.y,
                orientation.z,
            )
            yaw = quaternion.to_euler(degrees=False)[2]
        angle_to_goal = math.atan2(dy, dx) - yaw
        return (angle_to_goal + np.pi) % (2 * np.pi) - np.pi

    @staticmethod
    def observe_collision(lidar_data):
        min_lidar = np.min(lidar_data)
        if min_lidar < COLLISION_DIST:
            return True, True, min_lidar
        return False, False, min_lidar

    @staticmethod
    def get_reward(target, collision, action, min_lidar):
        if target:
            return 100.0
        elif collision:
            return -100.0
        else:
            r3 = lambda x: 1 - x if x < 1 else 0.0
            return action[0] / 2 - abs(action[1]) / 2 - r3(min_lidar) / 2
