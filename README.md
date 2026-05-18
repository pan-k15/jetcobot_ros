# 🦾 JetCobot ROS — 7-Axis Robotic Arm ROS 1 Workspace

A **ROS 1 Noetic** catkin workspace for the **Yahboom JetCobot** — a 7-axis visual collaborative robotic arm powered by NVIDIA Jetson. This repository provides URDF robot description, MoveIt motion planning configurations, and control nodes for both simulation and real hardware deployment.

---

## 🤖 Hardware Overview

| Spec | Details |
|---|---|
| **Robot** | Yahboom JetCobot 7-axis robotic arm |
| **Main Controller** | NVIDIA Jetson NANO 4GB / Orin NANO / Orin NX |
| **Degrees of Freedom** | 7-axis (UR-style configuration) |
| **Effective Arm Span** | 270 mm |
| **Joint Rotation Range** | −153° to +153° |
| **Camera** | 0.3MP USB, 110° FOV |
| **ROS Version** | ROS 1 Noetic (Ubuntu 20.04) |

---

## 📦 Packages

```
jetcobot_ros/
├── jetcobot_description/     # URDF / mesh files — robot model for RViz & Gazebo
├── jetcobot_control/         # Control nodes — joint commands, IK, motion execution (C++)
├── jetcobot_config/          # MoveIt configuration — planning groups, SRDF, kinematics (v1)
├── jetcobot_config_v2/       # MoveIt configuration — updated planning config (v2)
└── CMakeLists.txt            # Catkin workspace top-level CMake (symlink)
```

| Package | Description |
|---|---|
| `jetcobot_description` | URDF model, joint definitions, mesh assets for visualization and collision |
| `jetcobot_control` | ROS control nodes for joint trajectory execution and arm motion |
| `jetcobot_config` | MoveIt! setup — planning scene, SRDF, kinematics plugin config (v1) |
| `jetcobot_config_v2` | Updated MoveIt! configuration with revised planning parameters (v2) |

---

## 🛠️ Tech Stack

- **Framework:** ROS 1 Noetic
- **Build System:** catkin (`catkin_make`)
- **Motion Planning:** MoveIt!
- **Languages:** C++ (control), CMake (build), Python (utilities)
- **Simulation:** RViz, Gazebo
- **Vision (hardware-side):** OpenCV, Jetson-inference, MediaPipe

---

## 🚀 Getting Started

### Prerequisites

- Ubuntu 20.04
- ROS 1 Noetic (full desktop install)
- MoveIt 1 for Noetic

```bash
sudo apt install ros-noetic-desktop-full
sudo apt install ros-noetic-moveit
```

### 1. Create and set up your catkin workspace

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

### 2. Clone this repository into src/

```bash
git clone https://github.com/pan-k15/jetcobot_ros.git .
```

> The repo already contains the catkin workspace top-level `CMakeLists.txt` (symlink to catkin toplevel). Clone directly into `src/` or adjust paths accordingly.

### 3. Build the workspace

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 4. (Optional) Add to bashrc

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 🎯 Usage

### Launch robot description in RViz

```bash
roslaunch jetcobot_description display.launch
```

### Launch MoveIt with real hardware

```bash
roslaunch jetcobot_config demo.launch
```

### Launch MoveIt v2 configuration

```bash
roslaunch jetcobot_config_v2 demo.launch
```

### Run control node

```bash
rosrun jetcobot_control <node_name>
```

> Replace `<node_name>` with the specific executable from `jetcobot_control`. Use `rosrun jetcobot_control [Tab]` to list available nodes.

---

## 🧠 Capabilities

The JetCobot platform (combined hardware + ROS stack) supports:

- **Inverse Kinematics** — coordinate-based end-effector control via MoveIt
- **Motion Planning** — collision-aware trajectory planning with RRT/OMPL planners
- **MoveIt Simulation** — full simulation in RViz with joint trajectory execution
- **Pick & Place** — gripping and sorting via planned approach and grasp poses
- **Vision Integration** — OpenCV, Jetson-inference for color tracking, face detection, gesture control, label recognition
- **Multiple Control Modes** — MoveIt API, Android APP, gamepad/joystick, PC web interface

---

## ⚠️ Notes

- This is a **ROS 1 Noetic** workspace — not compatible with ROS 2 without bridging.
- `jetcobot_config` and `jetcobot_config_v2` are two iterations of the MoveIt configuration; prefer `v2` for updated kinematics parameters.
- The `CMakeLists.txt` at the root is a symlink to `/opt/ros/noetic/share/catkin/cmake/toplevel.cmake` — this is standard for catkin workspaces and does not need to be modified.
- Hardware deployment requires a Jetson board running Ubuntu 20.04 with JetPack.

---

## 🔗 References

- [Yahboom JetCobot Product Page](https://category.yahboom.net/products/jetcobot)
- [Yahboom JetCobot Official GitHub](https://github.com/YahboomTechnology/JetCobot)
- [ROS 1 Noetic Documentation](https://wiki.ros.org/noetic)
- [MoveIt 1 Documentation](https://moveit.ros.org/)

---

## 📄 License

This project is open source. Feel free to fork and extend.

---

## 👤 Author

**Pan** — [github.com/pan-k15](https://github.com/pan-k15)
