# ROS Integration Example Scripts

This directory provides conceptual and example ROS scripts developed during the Hybrid Control Framework project.  
These scripts demonstrate how to subscribe/publish to essential ROS topics, integrate VR data, perform teleoperation, reset odometry, and log robot data for later analysis.  
Each script is well-commented for learning and adaptation.

---

## 1. `odom_goal_nav.py`  
**Basic goal-seeking navigation using odometry.**

- **What it does:**  
  Subscribes to `/odom`, calculates angle to a fixed goal, and publishes `/cmd_vel` commands so the robot first rotates to face the goal, then moves forward.
- **Usage:**  
  Useful as a template for implementing classic "go to goal" behaviors on a real or simulated robot.
- **Key features:**  
  - Handles quaternion-to-Euler conversion.
  - Demonstrates angle-based movement logic.
  - Logs info with `rospy.loginfo`.

---

## 2. `vr_posrot_subscriber.py`  
**Subscribes to Unity VR pose data (`/pos_rot` topic with `PosRot` message).**

- **What it does:**  
  Listens for pose data sent from a Unity VR application (using the [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub) message definition) and logs it.
- **Usage:**  
  Serves as a bridge for using VR headset or avatar pose as an input for mobile robot navigation or teleoperation.
- **Key features:**  
  - Shows how to handle custom ROS messages (`PosRot`).
  - Ready for further logic (goal updating, live teleop, hybrid control).

---

## 3. `vr_posrot_teleop.py`  
**Direct teleoperation using VR pose deltas.**

- **What it does:**  
  Reads incremental changes in VR pose and triggers corresponding robot movements (forward/reverse or rotate).
- **Usage:**  
  Provides a foundation for real-time VR-based teleoperation ("mimicry" or parallel control).
- **Key features:**  
  - One movement command (forward, reverse, rotate left, rotate right) is sent per loop iteration for conceptual clarity.
  - **Combined control example included:** See the script’s docstring for how to modify the logic to allow simultaneous linear and angular motion.
  - Clearly comments and logs each triggered command.

---

## 4. `reset_odometry.py`  
**Utility for resetting TurtleBot3 odometry.**

- **What it does:**  
  Publishes to the `/reset` topic (`std_msgs/Empty`) to reset the robot's odometry (zero position/yaw), as supported by TurtleBot3 firmware and some simulators.
- **Usage:**  
  Useful for resetting between experimental runs, after manual repositioning, or when starting an evaluation.
- **Key features:**  
  - Attempts reset up to five times, with yaw check and waiting periods.
  - Shows how to subscribe to `/odom` and monitor pose state.

---

## 5. `odom_to_csv_logger.py`  
**Logs robot trajectory and velocity to CSV for later plotting/analysis.**

- **What it does:**  
  Subscribes to `/odom`, stores position, yaw, and velocity values, and continuously writes them to a CSV file.
- **Usage:**  
  Enables data-driven analysis of robot motion (trajectory plotting, experiment reproducibility).
- **Key features:**  
  - Outputs to `robot_trajectory_log.csv` by default.
  - Uses `pandas` for efficient data storage and writing.
  - Well-structured for quick analysis in Jupyter, Python, or Matlab.

---

## Requirements

- **ROS Noetic**
- Ubuntu 20.04 (recommended)
- **Python packages:**  
  - `pandas`  
  - `numpy`  
  Install with:
pip install pandas numpy

yaml
Copy
Edit
- **Custom messages:**  
- For VR/Unity scripts, build [unity_robotics_demo_msgs](https://github.com/Unity-Technologies/Unity-Robotics-Hub) in your catkin workspace.

---

## Usage

1. **Source your ROS workspace:**
source devel/setup.bash

markdown
Copy
Edit
2. **Run a script:**
rosrun ros_integration odom_goal_nav.py

yaml
Copy
Edit
(Replace with any script from this directory.)
3. **Check the logs or output files (e.g., CSV) as needed.**

---

## References

- [TurtleBot3 on ROS Wiki](http://wiki.ros.org/turtlebot3)
- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [ROS Navigation Stack](http://wiki.ros.org/navigation)

---


