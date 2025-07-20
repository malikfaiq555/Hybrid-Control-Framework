## 🟩 `README.md` for `drl_navigation/`

```markdown
# DRL Navigation Module (TD3, ROS, Gazebo)

This module enables Deep Reinforcement Learning navigation training and simulation in ROS/Gazebo, with **TD3** (Twin Delayed DDPG) and tested for TurtleBot3 Burger.  
**Flexible:** Swap in your own robot and Gazebo world by changing the launch file or environment settings!

---

## 🚀 Quick Overview

- **Train a DRL agent** to navigate to random goals in simulation (Gazebo + ROS).
- **Evaluate/test** your trained model in simulation.
- **Easily adapt** to your own launch file, world, or robot (just update the `launchfile` and topic names in `td3_env.py`).
- **ReplayBuffer** for stable off-policy learning.
- **Pre-trained weights** and checkpoints saved in `model_weights/`.

---

## 📂 Directory Structure


drl\_navigation/
├── README.md
├── scripts/
│   ├── train\_td3\_tb3.py         # Training script (TD3, editable)
│   ├── test\_td3\_tb3.py          # Evaluation script
│   ├── td3\_env.py               # Gazebo RL environment (customizable for your robot)
│   ├── replay\_buffer.py         # Experience replay buffer
│   └── ...                      # (You can add custom scripts/experiments here)
├── model\_weights/               # Saved models/checkpoints
│   ├── TD3\_tb3\_actor.pth
│   ├── TD3\_tb3\_critic.pth
│   └── ...
├── results/                     # Evaluation results, metrics
└── training\_demo.mp4            # (or link: see below)

````

---

## 🎥 Demo Video

See DRL training in action on TurtleBot3 in Gazebo:

- [![Training Demo Video](https://img.youtube.com/vi/XXXXXXXXXXX/0.jpg)](https://youtu.be/XXXXXXXXXXX)
- **OR**: Download or preview `training_demo.mp4` in this directory.

---

## 🛠️ Getting Started

### Requirements

- Python 3.8+
- PyTorch (`pip install torch`)
- ROS (Melodic/Noetic, tested on Ubuntu 18.04/20.04)
- Gazebo (with your robot/world)
- `numpy`, `squaternion`, and ROS Python dependencies

### 1. Edit Your Launch File & Topics

- By default, `multi_robot_scenario.launch` is used.  
- Change `launchfile` in `train_td3_tb3.py`, `test_td3_tb3.py`, and `td3_env.py` to match your world or robot.
- Edit lidar/pointcloud topic in `td3_env.py` if your robot uses a different topic.

### 2. Train the Agent

```bash
cd drl_navigation/conceptual_scripts/
python train_td3_tb3.py
````

* Models are saved to `../model_weights/`.
* Metrics and evaluations in `../results/`.

### 3. Test in Simulation

```bash
python test_td3_tb3.py
```

* This runs a trained policy in Gazebo.
* Edit the script to run for a set number of episodes or until you stop it.

### 4. Customization

* Change the **launch file** or world boundaries (`check_pos` in `td3_env.py`) for new environments.
* Update topics if your robot publishes odometry or pointcloud/lidar under different names.
* The state vector is `[lidar_points, distance_to_goal, angle_to_goal, lin_vel, ang_vel]` by default.

### 5. Pretrained Weights

* Place your weights as `TD3_tb3_actor.pth` and `TD3_tb3_critic.pth` in `model_weights/`.
* Or train from scratch as above.

---

## 🧠 How It Works

* **RL agent:** TD3 (twin delayed deep deterministic policy gradients)
* **Environment:** Goal is placed randomly, robot receives pointcloud/lidar input, odometry, computes reward.
* **Reward:** +100 for goal, -100 for collision, shaped for distance, heading, and smoothness.
* **Reset:** Robot and goal are randomized at episode start.

---

## 📌 References

* Algorithm: [Fujimoto et al., 2018 - TD3](https://arxiv.org/abs/1802.09477)
* Original code inspiration: [DRL-robot-navigation](https://github.com/reiniscimurs/DRL-robot-navigation)
* Customization for TurtleBot3 Burger with native 2D lidar.

---
