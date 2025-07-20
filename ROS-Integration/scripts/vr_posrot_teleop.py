#!/usr/bin/env python3
"""
ROS Node: vr_posrot_teleop

Description:
    Direct teleoperation of a real robot based on incremental pose changes
    from a VR robot, as published from Unity using the PosRot message.

    The script interprets deltas in VR robot position/orientation as commands:
        - Forward/Reverse: Linear X changes
        - Turn Left/Right: Yaw (theta) changes

    Only one movement command (forward, reverse, left, or right) is issued per control loop iteration.
    (This is for conceptual clarity—see "Combined Control" example in comments below if you want
    the robot to drive and turn at the same time.)

Topics:
    Subscribes:
        /odom    (nav_msgs/Odometry)     # For possible robot feedback (not used in control)
        /pos_rot (unity_robotics_demo_msgs/PosRot)  # VR robot pose
    Publishes:
        /cmd_vel (geometry_msgs/Twist)   # Velocity commands to the real robot

Combined Control Logic Example:
    # (Uncomment and use this instead of the loop below if you want simultaneous linear & angular movement)
    if inc_x > 0.0:
        speed.linear.x = 0.1
    elif inc_x < 0.0:
        speed.linear.x = -0.1
    else:
        speed.linear.x = 0.0

    if inc_theta > 0.0:
        speed.angular.z = -0.1
    elif inc_theta < 0.0:
        speed.angular.z = 0.1
    else:
        speed.angular.z = 0.0

    pub.publish(speed)

"""


import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from unity_robotics_demo_msgs.msg import PosRot

# Global variables for real robot odometry (not strictly used here, but could be extended)
x1 = 0.0
y1 = 0.0 
theta1 = 0.0

# VR robot pose, and previous pose for delta calculation
x2 = 0.0
y2 = 0.0 
theta2 = 0.0
prev_x2 = 0.0
prev_y2 = 0.0
prev_theta2 = 0.0

# Flags to indicate which command should be sent
forward_linear_change = False
reverse_linear_change = False
clockwise_angular_change = False
anticlockwise_angular_change = False
first_run = True

def odom_callback(msg):
    """
    Odometry callback (currently not used for feedback control).
    """
    global x1, y1, theta1
    x1 = msg.pose.pose.position.x
    y1 = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (_, _, theta1) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def posrot_callback(msg):
    """
    PosRot callback: Compares current and previous VR pose,
    sets flags for motion command to send.
    """
    global x2, y2, theta2
    global prev_x2, prev_y2, prev_theta2
    global forward_linear_change, reverse_linear_change
    global clockwise_angular_change, anticlockwise_angular_change
    global first_run

    # On first run, just record starting position
    if first_run:
        prev_x2 = msg.pos_z
        prev_y2 = -msg.pos_x
        prev_theta2 = msg.rot_y
        first_run = False

    x2 = msg.pos_z        # VR convention: pos_z becomes robot x
    y2 = -msg.pos_x       # VR convention: -pos_x becomes robot y
    theta2 = msg.rot_y    # VR convention: rot_y is robot yaw

    inc_x = x2 - prev_x2
    inc_theta = theta2 - prev_theta2

    # Linear movement commands (forward/reverse)
    if inc_x > 0.0:
        forward_linear_change = True
    elif inc_x < 0.0:
        reverse_linear_change = True

    # Angular movement commands (yaw change)
    if inc_theta > 0.0:
        clockwise_angular_change = True
    elif inc_theta < 0.0:
        anticlockwise_angular_change = True

    # Update previous pose for next comparison
    prev_x2 = x2
    prev_y2 = y2
    prev_theta2 = theta2

def main():
    rospy.init_node("vr_posrot_teleop")

    # Subscribers
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/pos_rot", PosRot, posrot_callback)
    # Publisher
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    speed = Twist()
    rate = rospy.Rate(3)  # 3 Hz

    while not rospy.is_shutdown():
        # Decide which command to send based on which flag is set
        if forward_linear_change:
            rospy.loginfo("Forward command triggered from VR.")
            speed.linear.x = 0.1
            speed.angular.z = 0.0
            forward_linear_change = False
        elif reverse_linear_change:
            rospy.loginfo("Reverse command triggered from VR.")
            speed.linear.x = -0.1
            speed.angular.z = 0.0
            reverse_linear_change = False    
        elif clockwise_angular_change:
            rospy.loginfo("Clockwise turn command triggered from VR.")
            speed.linear.x = 0.0
            speed.angular.z = -0.1
            clockwise_angular_change = False
        elif anticlockwise_angular_change:
            rospy.loginfo("Anticlockwise turn command triggered from VR.")
            speed.linear.x = 0.0
            speed.angular.z = 0.1
            anticlockwise_angular_change = False
        else:
            speed.linear.x = 0.0
            speed.angular.z = 0.0
            
            
            # # --- Combined Control Example ---
        # # Uncomment below for combined linear and angular movement in one loop
        # speed.linear.x = 0.0
        # speed.angular.z = 0.0
        # if forward_linear_change:
        #     speed.linear.x = 0.1
        #     forward_linear_change = False
        # elif reverse_linear_change:
        #     speed.linear.x = -0.1
        #     reverse_linear_change = False
        # if clockwise_angular_change:
        #     speed.angular.z = -0.1
        #     clockwise_angular_change = False
        # elif anticlockwise_angular_change:
        #     speed.angular.z = 0.1
        #     anticlockwise_angular_change = False
        # pub.publish(speed)


        pub.publish(speed)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
