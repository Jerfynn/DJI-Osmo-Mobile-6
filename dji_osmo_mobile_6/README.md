# DJI Osmo Mobile 6 - ROS 2 & Gazebo Simulation Package

This package contains the complete simulation setup for the **DJI Osmo Mobile 6** 3-axis handheld gimbal, converted from the high-resolution `.gltf` model with exact mechanical pivot points, inertial tensors, and a phone-mounted IMU sensor for active stabilization.

---

## Kinematic Chain & Axes

```
[base_link] (Grip handle & controls)
    │
    ▼ yaw_joint (Pan axis, Z = [0, 0, 1] at Z = 0.107m)
[yaw_link] (Pan arm)
    │
    ▼ roll_joint (Roll axis with 23° sweep, at Z = 0.135m)
[roll_link] (Roll arm & tilt motor bracket)
    │
    ▼ pitch_joint (Tilt axis, Y = [0, 1, 0] at Z = 0.194m)
[pitch_link] (Magnetic phone clamp + mounted phone payload)
    │
    ▼ imu_joint (Fixed mount at phone center)
[imu_link] (Gazebo IMU sensor)
```

---

## Package Structure

```
dji_osmo_mobile_6/
├── CMakeLists.txt
├── package.xml
├── urdf/
│   ├── osmo_mobile_6.urdf        # Pure URDF file
│   └── osmo_mobile_6.urdf.xacro  # Parametric Xacro file
├── meshes/
│   ├── dae/                      # Collada meshes (base_link, yaw_link, roll_link, pitch_link)
│   └── stl/                      # High-speed binary STL meshes
├── launch/
│   ├── display.launch.py         # Launch in RViz2 with GUI joint sliders
│   └── gazebo.launch.py          # Spawn robot in Gazebo simulation
├── config/
│   └── controllers.yaml          # Joint controllers configuration
└── scripts/
    └── gimbal_stabilizer.py      # Closed-loop PID stabilization controller
```

---

## How to Build & Run

### 1. Build Package in ROS 2 Workspace
```bash
# In your ROS 2 colcon workspace (e.g. ~/ros2_ws)
colcon build --packages-select dji_osmo_mobile_6
source install/setup.bash
```

### 2. Preview Kinematics in RViz2 (Joint Slider GUI)
```bash
ros2 launch dji_osmo_mobile_6 display.launch.py
```
Move the GUI sliders to verify that Yaw, Roll, and Pitch rotate cleanly around their respective pivots.

### 3. Launch in Gazebo
```bash
ros2 launch dji_osmo_mobile_6 gazebo.launch.py
```

### 4. Run the Active IMU Stabilization Loop
```bash
ros2 run dji_osmo_mobile_6 gimbal_stabilizer.py
```
This node subscribes to `/gimbal/imu/data` from the phone sensor and issues corrective commands to keep the phone horizontal and level.
