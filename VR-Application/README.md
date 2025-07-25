This directory contains scripts and components designed for integrating ROS (Robot Operating System) functionalities within a Unity-based Virtual Reality (VR) application. It demonstrates core ROS concepts such as subscribing to sensor topics, publishing robot state, visualizing data, and interacting with VR controllers.

---

## Directory Structure

```
VR_Application/
├── README.md
├── scripts/
│   ├── CompressedImageRenderer.cs
│   ├── RobotInfoDisplay.cs
│   ├── RobotMotionSync.cs
│   ├── LaserScanVisualizer.cs
│   ├── ROSPosePublisher.cs
│   └── VRRobotController.cs
└── assets/
```

---

## ⚙️ Setup Requirements

- **Unity Engine** (version: **2021 LTS**)
- **Meta Quest 2** headset (formerly Oculus Quest 2)
- [**Oculus Integration SDK**](https://developer.oculus.com/downloads/package/unity-integration/) for Unity
- [**Unity Robotics Hub**](https://github.com/Unity-Technologies/Unity-Robotics-Hub) for ROS-Unity communication
- **ROS Noetic** environment configured (see [ROS Setup](../ros_integration/README.md))


---

## Usage Overview

### 1. ROSCompressedImageSubscriber.cs

* **Purpose**:
  Subscribes to ROS compressed image topics and renders received images onto a Unity GameObject.

* **How to Use**:

  * Attach to an empty GameObject in Unity.
  * Set ROS image topic name and target display object in Unity Inspector.

---

### 2. RobotInfoDisplay.cs

* **Purpose**:
  Displays real-time robot heading and positional data.

* **How to Use**:

  * Attach directly to your virtual robot GameObject.
  * Assign TextMesh objects for displaying heading and position.

---

### 3. RobotMotionSync.cs

* **Purpose**:
  Syncs a virtual robot's movement in Unity with actual robot odometry and velocity data from ROS.

* **How to Use**:

  * Attach script to your virtual robot GameObject.
  * Ensure proper ROS topic subscriptions for odometry (`/odom`) and velocity (`/cmd_vel`).
  * Customize VR inputs and ROS topics through Unity Inspector.
  * Currently sends predefined goal positions; see below for dynamic interaction options.

---

### 4. LaserScanVisualizer.cs

* **Purpose**:
  Visualizes ROS LaserScan messages as a clear and simplified point cloud overlay within a VR environment, enhancing spatial awareness.

* **Visualization Clarification**:
  Currently uses Unity `LineRenderer` components to connect sparse laser scan points for clear obstacle representation.

* **Advanced Visualization Alternatives**:
  For dense point cloud visualization, consider using specialized Unity plugins:

  * [Pcx](https://github.com/keijiro/Pcx) for `.ply` point clouds.
  * [FastPoints](https://github.com/eliasnd/FastPoints) for large-scale point cloud management.

* **How to Use**:

  * Attach this script to an empty GameObject in your VR scene.
  * Adjust settings such as topic name, brightness, and color in the Inspector.

---

### 5. ROSPosePublisher.cs

* **Purpose**:
  Demonstrates publishing a virtual robot’s pose (position & orientation) from Unity to ROS.

* **ROS Message Type**:
  Uses the `PosRotMsg` type provided in the [Unity-Robotics-Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub).

* **How to Use**:

  * Attach script to any Unity GameObject whose pose data you want to publish.
  * Configure publishing frequency and topic through the Inspector.

---

### 6. VRRobotController.cs

* **Purpose**:
  Enables direct user control of the virtual robot using VR controller inputs.

* **How to Use**:

  * Attach directly to your virtual robot GameObject or an empty parent GameObject.
  * Adjust rotation and movement speeds via Inspector.
  * Ensure correct input axes mapping based on your VR hardware.

---

## Dynamic Interaction and Customization

* **Static vs. Dynamic Goals**:
  Currently, robot goals and waypoints are predefined. Users may consider adding dynamic goal-setting functionality through VR interactions such as:

  * VR pointers/laser pointers for selecting waypoints at runtime.
  * Custom UI elements or voice commands to define robot goal locations.

* **Customization Suggestions**:

  * Adjust movement parameters (speed, rotation) and sensor visualizations based on your robot and environment.
  * Incorporate additional ROS sensors/topics following provided examples.

---

##  Useful Resources & References

* [Unity Robotics Hub Official Repository](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
* [ROS Official Documentation](https://wiki.ros.org/)
* [Unity VR Development Guide](https://docs.unity3d.com/Manual/XR.html)
* [Pcx: Point Cloud Importer/Renderer for Unity](https://github.com/keijiro/Pcx)
* [FastPoints: Unity Point Cloud Renderer](https://github.com/eliasnd/FastPoints)

---

## Notes

* Ensure your ROS master and Unity instances are configured correctly and connected to the same network.
* Carefully verify topic names and message types in both ROS and Unity scripts.
* Feel free to extend and adapt these base scripts to fit your project's specific requirements.

---


