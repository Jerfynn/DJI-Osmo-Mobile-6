# 📷 DJI Osmo Mobile 6 - ROS 2 & Gazebo Simulation

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble%20%7C%20Iron%20%7C%20Jazzy-blue?logo=ros)](https://docs.ros.org/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Classic%20%7C%20Ignition%20%7C%20Gz-orange)](https://gazebosim.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)

A complete simulation package for the **DJI Osmo Mobile 6** 3-axis smartphone handheld gimbal in **ROS 2** and **Gazebo**. 

This repository converts a high-fidelity 3D model into an articulated robotic kinematic chain featuring:
- Accurate joint pivot placements and rotational limits
- 23° forward-swept ergonomic roll arm alignment
- Calculated mass and $3\times3$ moment of inertia tensors
- Simulated phone-mounted IMU sensor (200 Hz with realistic MEMS noise)
- Real-time closed-loop PID stabilization controller

---

## 📐 Kinematic Structure & Joint Hierarchy

```
[world]
   │ (fixed base / tripod)
[base_link] (Grip handle, battery, controls & status display)
   │
   ▼ yaw_joint (Pan axis: Z = [0, 0, 1] at Z = 0.107m)
[yaw_link] (Pan arm & roll motor casing)
   │
   ▼ roll_joint (Roll axis: [0.9205, 0.3908, 0] with 23° sweep at Z = 0.135m)
[roll_link] (Roll arm & tilt motor bracket)
   │
   ▼ pitch_joint (Tilt axis: Y = [0, 1, 0] at Z = 0.194m)
[pitch_link] (Magnetic clamp bracket + smartphone payload)
   │
   ▼ imu_joint (Rigid mount at smartphone payload center)
[imu_link] (Gazebo IMU sensor)
```

---

## 📁 Repository Structure

```
DJI-Osmo-Mobile-6/
├── dji_osmo_mobile_6/            # ROS 2 package
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── README.md
│   ├── urdf/
│   │   ├── osmo_mobile_6.urdf        # Full standalone URDF with Gazebo plugins
│   │   └── osmo_mobile_6.urdf.xacro  # Parametric Xacro model
│   ├── meshes/
│   │   ├── stl/                      # Lightweight binary STL meshes
│   │   └── dae/                      # Textured Collada meshes
│   ├── launch/
│   │   ├── display.launch.py         # Visual preview in RViz2 with joint sliders
│   │   └── gazebo.launch.py          # Spawn gimbal model in Gazebo
│   ├── config/
│   │   └── controllers.yaml          # Position/Effort joint controllers
│   └── scripts/
│       └── gimbal_stabilizer.py      # Real-time closed-loop IMU stabilization
├── source/                           # Original raw glTF 3D model and textures
├── generate_gimbal_package.py        # Automated python pipeline for mesh extraction
├── .gitignore
└── README.md
```

---

## 🚀 Getting Started

### Prerequisites
- ROS 2 (Humble, Iron, or Jazzy)
- Gazebo (`gazebo_ros_pkgs` or Ignition / Gz Sim)
- Python 3 (`numpy`, `trimesh`)

### 1. Build the Package
Clone this repository into your ROS 2 workspace `src/` folder:
```bash
cd ~/ros2_ws/src
git clone https://github.com/Jerfynn/DJI-Osmo-Mobile-6.git
cd ~/ros2_ws
colcon build --packages-select dji_osmo_mobile_6
source install/setup.bash
```

### 2. Preview Kinematics in RViz2
Launch the model with the interactive joint state GUI:
```bash
ros2 launch dji_osmo_mobile_6 display.launch.py
```
Use the sliders in the GUI window to test articulation across Pan (Yaw), Roll, and Tilt (Pitch).

### 3. Launch Simulation in Gazebo
Spawn the gimbal in a physics world:
```bash
ros2 launch dji_osmo_mobile_6 gazebo.launch.py
```

### 4. Run Active IMU Stabilization
Start the closed-loop PID controller:
```bash
ros2 run dji_osmo_mobile_6 gimbal_stabilizer.py
```
The node subscribes to `/gimbal/imu/data` from the simulated phone payload sensor and dynamically generates joint effort commands to keep the phone horizontal and level against disturbances.

---

## ⚙️ Specifications

| Parameter | Value |
| :--- | :--- |
| **Gimbal Mass** | ~0.340 kg |
| **Payload Mass** | ~0.180 kg (Smartphone) |
| **Total System Mass** | ~0.520 kg |
| **Yaw Joint Range** | -160° to +160° |
| **Roll Joint Range** | -90° to +90° |
| **Pitch Joint Range** | -100° to +100° |
| **IMU Update Rate** | 200 Hz |

---

## 📜 License
This project is licensed under the Apache 2.0 License.
