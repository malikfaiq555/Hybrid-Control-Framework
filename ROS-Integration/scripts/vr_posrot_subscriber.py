#!/usr/bin/env python3
"""
ROS Node: vr_posrot_subscriber
Description: 
    Subscribes to a custom message type (`PosRot` from unity_robotics_demo_msgs)
    published by Unity or the  VR app and logs the position data.

    This demonstrates how to integrate external pose/goal information into ROS
    for use in direct teleoperation through parallel supervisory control, global goal setting etc.
Topics:
    Subscribes: /pos_rot (unity_robotics_demo_msgs/PosRot)
"""

import rospy
from geometry_msgs.msg import Point
from unity_robotics_demo_msgs.msg import PosRot

# Global variables to store VR pose data
x_vr = 0.0
y_vr = 0.0
z_vr = 0.0
# (Optionally, orientation can be added if provided in the message)

def posrot_callback(msg):
    """
    Callback for /pos_rot topic.
    Logs incoming VR pose data.
    """
    global x_vr, y_vr, z_vr

    x_vr = msg.pos_x
    y_vr = msg.pos_y
    z_vr = msg.pos_z  # If not present, you can remove this line.

    rospy.loginfo(f"Received VR Pose: x={x_vr:.2f}, y={y_vr:.2f}, z={z_vr:.2f}")

    # If needed, you can add logic here to process or forward this data.

def main():
    rospy.init_node("vr_posrot_subscriber")

    # Subscribe to the /pos_rot topic (custom message type)
    rospy.Subscriber("/pos_rot", PosRot, posrot_callback)

    rate = rospy.Rate(4)  # 4 Hz

    while not rospy.is_shutdown():
        # Here, we simply log the current goal (VR pose)
        rospy.loginfo(f"Current VR Goal: x={x_vr:.2f}, y={y_vr:.2f}, z={z_vr:.2f}")
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
