---
# Real Robot Deployment — TurtleBot3 + TD3 Navigation

This module lets you deploy and test your trained **TD3 navigation policy** on a real TurtleBot3 Burger via ROS!  
You can send goals from VR, RViz, or a teleop interface, and watch your robot navigate autonomously.

---

##  Directory Contents

```

Real-Robot-Deployment/
├── README.md
├── scripts/
│   ├── real_env.py                 # Real-world TurtleBot3 environment (ROS)
│   └──  test_td3_tb3_real.py       # Script: run trained model on real robot
│                           
└──  model_weights/
    ├── TD3_tb3_actor.pth
    └── TD3_tb3_critic.pth


```

---

## Setup & Requirements

* **Hardware** TurtleBot3 Burger (or any ROS 1 robot with 2-D LiDAR + odometry)  
* **Software** ROS Noetic / Melodic, Python 3.8+, PyTorch, `numpy`, `squaternion`  
* **Network** Robot and PC on the **same subnet** (tested on a single Wi-Fi AP).  

> **Different networks?** See the _Networking guide_ below.

---

## Multi-Machine ROS (same LAN)

On the **PC (ROS master)**

```bash
export ROS_MASTER_URI=http://<PC_IP>:11311
export ROS_HOSTNAME=<PC_IP>
roscore
````

On the **TurtleBot3**

```bash
export ROS_MASTER_URI=http://<PC_IP>:11311
export ROS_HOSTNAME=<ROBOT_IP>
```

Check connectivity with `rostopic list` from each machine.
(ROS uses TCPROS on port 11311 for the master and dynamic ports for node-to-node traffic).

---

## Networking guide (different subnets / the public internet)

| Option                          | What to do                                                                                                                                                                                     | Notes                                                                                                                                     |
| ------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------- |
| **Static IP + Port Forwarding** | Forward **port 11311** (ROS master) **and a range** of high ports (e.g. 50000-60000) from your router to the PC running `roscore`. Then set `ROS_TCP_PORT_RANGE=50000-60000` on every machine. | Works but scales poorly; every ROS node still opens random ports that must be reachable ([Robotics Stack Exchange][2], [ROS Answers][3]). |
| **VPN (preferred)**             | Use ZeroTier, WireGuard, or Tailscale to put the PC and robot in the **same virtual subnet**. Then follow the standard multi-machine steps above.                                              | No port juggling; behaves like a local LAN.                                                                                               |
| **SSH tunnel / rosbridge**      | Tunnel only the topics you need (`rosbridge_websocket` or `ssh -L`) if full ROS graph isn’t required.                                                                                          | Good for dashboards or lightweight teleop.                                                                                                |

---

## How to Run

1. Copy the trained weights to `model_weights/`:

   ```
   TD3_tb3_actor.pth
   TD3_tb3_critic.pth
   ```
2. On the robot:

   ```bash
   roslaunch turtlebot3_bringup turtlebot3_robot.launch
   ```
3. (Optional) Start RViz, VR‐goal publisher, or any teleop node.
4. On the PC:

   ```bash
   cd real_robot_deployment/scripts
   python3 test_td3_tb3_real.py
   ```
5. Send a goal; the robot should drive autonomously.

---

## Safety Checklist

* Supervise the robot at all times.
* Start in an open space and increase complexity gradually.
* Keep an emergency-stop key or power switch within reach.

---

## Customization

* **Different robot** Edit topic names in `real_env.py`.
* **Goal interface** `test_td3_tb3_real.py` expects a custom `/pos_rot` goal topic; adapt as needed.

---

## References

* ROS multi-machine tutorial  ([ROS Wiki][1])
* Port requirements for ROS nodes  ([Robotics Stack Exchange][2])
* ROS behind NAT discussion  ([ROS Answers][3])

---


[1]: https://wiki.ros.org/ROS/Tutorials/MultipleMachines?utm_source=chatgpt.com "ROS/Tutorials/MultipleMachines - ROS Wiki"
[2]: https://robotics.stackexchange.com/questions/64220/which-ports-are-needed-for-ros?utm_source=chatgpt.com "which ports are needed for ROS? - Robotics Stack Exchange"
[3]: https://answers.ros.org/question/364321/?utm_source=chatgpt.com "ROS Networking question: Strategies for reaching a roscore behind a NAT ..."
