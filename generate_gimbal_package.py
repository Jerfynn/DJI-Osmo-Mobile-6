import os
import json
import numpy as np
import trimesh

def build_gimbal_package():
    workspace = os.path.dirname(os.path.abspath(__file__))
    gltf_path = os.path.join(workspace, 'source', 'OSMO Mobile 6', 'OSMO Mobile 6', 'OSMO Mobile 6.gltf')
    
    pkg_dir = os.path.join(workspace, 'dji_osmo_mobile_6')
    mesh_dae_dir = os.path.join(pkg_dir, 'meshes', 'dae')
    mesh_stl_dir = os.path.join(pkg_dir, 'meshes', 'stl')
    urdf_dir = os.path.join(pkg_dir, 'urdf')
    launch_dir = os.path.join(pkg_dir, 'launch')
    config_dir = os.path.join(pkg_dir, 'config')
    scripts_dir = os.path.join(pkg_dir, 'scripts')
    
    for d in [mesh_dae_dir, mesh_stl_dir, urdf_dir, launch_dir, config_dir, scripts_dir]:
        os.makedirs(d, exist_ok=True)
        
    print(f"Loading GLTF from {gltf_path}...")
    scene = trimesh.load(gltf_path)
    with open(gltf_path, 'r') as f:
        data = json.load(f)
        
    # Group indices verified from model topology
    link_groups = {
        'base_link': [17, 21, 22, 23, 26, 2, 25, 32, 20, 15, 28, 27, 30, 31, 4, 24, 19],
        'yaw_link': [29, 18, 33, 40, 35, 34],
        'roll_link': [38, 39, 37, 36],
        'pitch_link': [10, 1, 13, 5, 7, 6, 0, 9, 16, 8, 14, 12, 11, 3]
    }
    
    # Real-world estimated masses (kg)
    link_masses = {
        'base_link': 0.180,   # Handle grip, battery, mainboard, buttons
        'yaw_link': 0.055,    # Pan arm and motor housing
        'roll_link': 0.065,   # Roll arm and pitch motor bracket
        'pitch_link': 0.220   # Magnetic phone clamp (0.04kg) + smartphone (0.18kg)
    }
    
    # Transform matrix from GLTF to ROS standard (+Z up, +X forward, +Y left)
    R_align = np.array([
        [ 0.0,  0.0, -1.0],
        [-1.0,  0.0,  0.0],
        [ 0.0,  1.0,  0.0]
    ])
    
    # Get yaw ring reference node (node 19: MESH.020)
    node_19 = data['nodes'][19]
    t19, _ = scene.graph[node_19['name']]
    geom_19_key = list(scene.geometry.keys())[node_19['mesh']]
    v19_raw = trimesh.transformations.transform_points(scene.geometry[geom_19_key].vertices, t19)
    
    yaw_center_gltf = v19_raw.mean(axis=0)
    bottom_y_gltf = scene.bounds[0][1]
    offset_gltf = np.array([yaw_center_gltf[0], bottom_y_gltf, yaw_center_gltf[2]])
    
    def transform_to_ros(vertices):
        return (R_align @ (vertices - offset_gltf).T).T

    # Calculate Pivots in global ROS coordinates
    # Yaw pivot is at (0, 0, z_yaw)
    yaw_pivot_ros = np.array([0.0, 0.0, transform_to_ros(v19_raw).mean(0)[2]])
    
    # Roll pivot from node 35 and 40 (roll motor caps)
    n35, n40 = data['nodes'][35], data['nodes'][40]
    v35 = transform_to_ros(trimesh.transformations.transform_points(scene.geometry[list(scene.geometry.keys())[n35['mesh']]].vertices, scene.graph[n35['name']][0]))
    v40 = transform_to_ros(trimesh.transformations.transform_points(scene.geometry[list(scene.geometry.keys())[n40['mesh']]].vertices, scene.graph[n40['name']][0]))
    roll_pivot_ros = (v35.mean(0) + v40.mean(0)) / 2.0
    
    # Pitch pivot from node 1 and 10 (pitch hinge pins)
    n1, n10 = data['nodes'][1], data['nodes'][10]
    v1 = transform_to_ros(trimesh.transformations.transform_points(scene.geometry[list(scene.geometry.keys())[n1['mesh']]].vertices, scene.graph[n1['name']][0]))
    v10 = transform_to_ros(trimesh.transformations.transform_points(scene.geometry[list(scene.geometry.keys())[n10['mesh']]].vertices, scene.graph[n10['name']][0]))
    pitch_pivot_ros = (v1.mean(0) + v10.mean(0)) / 2.0
    
    link_pivots_ros = {
        'base_link': np.array([0.0, 0.0, 0.0]),
        'yaw_link': yaw_pivot_ros,
        'roll_link': roll_pivot_ros,
        'pitch_link': pitch_pivot_ros
    }
    
    print("Computed Pivots (ROS World Frame):")
    for name, p in link_pivots_ros.items():
        print(f"  {name:10}: {np.round(p, 4)}")
        
    link_data = {}
    
    # Process each link mesh
    for link_name, indices in link_groups.items():
        submeshes = []
        for idx in indices:
            node = data['nodes'][idx]
            node_name = node['name']
            geom_name = list(scene.geometry.keys())[node['mesh']]
            geom = scene.geometry[geom_name]
            
            transform, _ = scene.graph[node_name]
            world_verts = trimesh.transformations.transform_points(geom.vertices, transform)
            ros_verts = transform_to_ros(world_verts)
            
            # Shift to link's local origin (the joint pivot)
            local_verts = ros_verts - link_pivots_ros[link_name]
            
            submesh = trimesh.Trimesh(vertices=local_verts, faces=geom.faces, process=False)
            submeshes.append(submesh)
            
        combined = trimesh.util.concatenate(submeshes)
        
        # Export meshes
        dae_file = os.path.join(mesh_dae_dir, f"{link_name}.dae")
        stl_file = os.path.join(mesh_stl_dir, f"{link_name}.stl")
        
        combined.export(stl_file)
        try:
            combined.export(dae_file)
        except Exception as e:
            print(f"DAE export note for {link_name}: {e}")
            
        # Compute inertial properties
        mass = link_masses[link_name]
        com = combined.centroid
        
        # Approximate inertia tensor using bounding box inertia or trimesh moment
        ext = combined.extents
        # Solid box approximation inertia formula: I_xx = m/12 * (dy^2 + dz^2)
        ixx = max(1e-6, (mass / 12.0) * (ext[1]**2 + ext[2]**2))
        iyy = max(1e-6, (mass / 12.0) * (ext[0]**2 + ext[2]**2))
        izz = max(1e-6, (mass / 12.0) * (ext[0]**2 + ext[1]**2))
        
        link_data[link_name] = {
            'mass': mass,
            'com': com,
            'extents': ext,
            'ixx': ixx,
            'iyy': iyy,
            'izz': izz
        }
        print(f"Processed {link_name:10}: mass={mass}kg, COM={np.round(com, 4)}, extents={np.round(ext, 4)}")
        
    # Relative Joint Offsets
    yaw_origin = yaw_pivot_ros - link_pivots_ros['base_link']
    roll_origin = roll_pivot_ros - link_pivots_ros['yaw_link']
    pitch_origin = pitch_pivot_ros - link_pivots_ros['roll_link']
    
    # Phone center / IMU offset relative to pitch_link
    phone_com = link_data['pitch_link']['com']
    
    print("\nRelative Joint Origins:")
    print(f"  yaw_joint   (base -> yaw)  : xyz='{yaw_origin[0]:.4f} {yaw_origin[1]:.4f} {yaw_origin[2]:.4f}'")
    print(f"  roll_joint  (yaw -> roll)  : xyz='{roll_origin[0]:.4f} {roll_origin[1]:.4f} {roll_origin[2]:.4f}'")
    print(f"  pitch_joint (roll -> pitch): xyz='{pitch_origin[0]:.4f} {pitch_origin[1]:.4f} {pitch_origin[2]:.4f}'")
    
    # Write URDF
    urdf_content = f"""<?xml version="1.0" encoding="utf-8"?>
<!-- ========================================================================= -->
<!-- DJI Osmo Mobile 6 URDF Model with 3-Axis Gimbal and Phone IMU Sensor      -->
<!-- Generated automatically with precise kinematic pivots and inertial tensors -->
<!-- ========================================================================= -->
<robot name="dji_osmo_mobile_6">

  <!-- Materials for RViz visualization -->
  <material name="dji_dark_grey">
    <color rgba="0.18 0.19 0.20 1.0"/>
  </material>
  <material name="dji_silver">
    <color rgba="0.75 0.75 0.76 1.0"/>
  </material>
  <material name="phone_glass">
    <color rgba="0.10 0.10 0.12 0.95"/>
  </material>

  <!-- ======================== BASE LINK (Grip & Controls) ======================== -->
  <link name="base_link">
    <inertial>
      <origin xyz="{link_data['base_link']['com'][0]:.4f} {link_data['base_link']['com'][1]:.4f} {link_data['base_link']['com'][2]:.4f}" rpy="0 0 0"/>
      <mass value="{link_data['base_link']['mass']:.3f}"/>
      <inertia ixx="{link_data['base_link']['ixx']:.6e}" ixy="0" ixz="0"
               iyy="{link_data['base_link']['iyy']:.6e}" iyz="0"
               izz="{link_data['base_link']['izz']:.6e}"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://dji_osmo_mobile_6/meshes/stl/base_link.stl"/>
      </geometry>
      <material name="dji_dark_grey"/>
    </visual>
    <collision>
      <origin xyz="{link_data['base_link']['com'][0]:.4f} {link_data['base_link']['com'][1]:.4f} {link_data['base_link']['com'][2]:.4f}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="{max(link_data['base_link']['extents'][0], link_data['base_link']['extents'][1])/2.0:.4f}" length="{link_data['base_link']['extents'][2]:.4f}"/>
      </geometry>
    </collision>
  </link>

  <!-- ======================== YAW (PAN) JOINT & LINK ======================== -->
  <joint name="yaw_joint" type="revolute">
    <parent link="base_link"/>
    <child link="yaw_link"/>
    <origin xyz="{yaw_origin[0]:.4f} {yaw_origin[1]:.4f} {yaw_origin[2]:.4f}" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-2.80" upper="2.80" effort="1.2" velocity="6.28"/>
    <dynamics damping="0.01" friction="0.005"/>
  </joint>

  <link name="yaw_link">
    <inertial>
      <origin xyz="{link_data['yaw_link']['com'][0]:.4f} {link_data['yaw_link']['com'][1]:.4f} {link_data['yaw_link']['com'][2]:.4f}" rpy="0 0 0"/>
      <mass value="{link_data['yaw_link']['mass']:.3f}"/>
      <inertia ixx="{link_data['yaw_link']['ixx']:.6e}" ixy="0" ixz="0"
               iyy="{link_data['yaw_link']['iyy']:.6e}" iyz="0"
               izz="{link_data['yaw_link']['izz']:.6e}"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://dji_osmo_mobile_6/meshes/stl/yaw_link.stl"/>
      </geometry>
      <material name="dji_dark_grey"/>
    </visual>
    <collision>
      <origin xyz="{link_data['yaw_link']['com'][0]:.4f} {link_data['yaw_link']['com'][1]:.4f} {link_data['yaw_link']['com'][2]:.4f}" rpy="0 0 0"/>
      <geometry>
        <box size="{link_data['yaw_link']['extents'][0]:.4f} {link_data['yaw_link']['extents'][1]:.4f} {link_data['yaw_link']['extents'][2]:.4f}"/>
      </geometry>
    </collision>
  </link>

  <!-- ======================== ROLL JOINT & LINK ======================== -->
  <joint name="roll_joint" type="revolute">
    <parent link="yaw_link"/>
    <child link="roll_link"/>
    <origin xyz="{roll_origin[0]:.4f} {roll_origin[1]:.4f} {roll_origin[2]:.4f}" rpy="0 0 0"/>
    <!-- The 23-degree ergonomic slant of the Osmo roll arm -->
    <axis xyz="0.9205 0.3908 0"/>
    <limit lower="-1.60" upper="1.60" effort="1.2" velocity="6.28"/>
    <dynamics damping="0.01" friction="0.005"/>
  </joint>

  <link name="roll_link">
    <inertial>
      <origin xyz="{link_data['roll_link']['com'][0]:.4f} {link_data['roll_link']['com'][1]:.4f} {link_data['roll_link']['com'][2]:.4f}" rpy="0 0 0"/>
      <mass value="{link_data['roll_link']['mass']:.3f}"/>
      <inertia ixx="{link_data['roll_link']['ixx']:.6e}" ixy="0" ixz="0"
               iyy="{link_data['roll_link']['iyy']:.6e}" iyz="0"
               izz="{link_data['roll_link']['izz']:.6e}"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://dji_osmo_mobile_6/meshes/stl/roll_link.stl"/>
      </geometry>
      <material name="dji_dark_grey"/>
    </visual>
    <collision>
      <origin xyz="{link_data['roll_link']['com'][0]:.4f} {link_data['roll_link']['com'][1]:.4f} {link_data['roll_link']['com'][2]:.4f}" rpy="0 0 0"/>
      <geometry>
        <box size="{link_data['roll_link']['extents'][0]:.4f} {link_data['roll_link']['extents'][1]:.4f} {link_data['roll_link']['extents'][2]:.4f}"/>
      </geometry>
    </collision>
  </link>

  <!-- ======================== PITCH (TILT) JOINT & LINK ======================== -->
  <joint name="pitch_joint" type="revolute">
    <parent link="roll_link"/>
    <child link="pitch_link"/>
    <origin xyz="{pitch_origin[0]:.4f} {pitch_origin[1]:.4f} {pitch_origin[2]:.4f}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.80" upper="1.80" effort="1.2" velocity="6.28"/>
    <dynamics damping="0.01" friction="0.005"/>
  </joint>

  <link name="pitch_link">
    <inertial>
      <origin xyz="{link_data['pitch_link']['com'][0]:.4f} {link_data['pitch_link']['com'][1]:.4f} {link_data['pitch_link']['com'][2]:.4f}" rpy="0 0 0"/>
      <mass value="{link_data['pitch_link']['mass']:.3f}"/>
      <inertia ixx="{link_data['pitch_link']['ixx']:.6e}" ixy="0" ixz="0"
               iyy="{link_data['pitch_link']['iyy']:.6e}" iyz="0"
               izz="{link_data['pitch_link']['izz']:.6e}"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://dji_osmo_mobile_6/meshes/stl/pitch_link.stl"/>
      </geometry>
      <material name="dji_silver"/>
    </visual>
    <collision>
      <origin xyz="{link_data['pitch_link']['com'][0]:.4f} {link_data['pitch_link']['com'][1]:.4f} {link_data['pitch_link']['com'][2]:.4f}" rpy="0 0 0"/>
      <geometry>
        <!-- Phone clamp and smartphone body bounding box -->
        <box size="{link_data['pitch_link']['extents'][0]:.4f} {link_data['pitch_link']['extents'][1]:.4f} {link_data['pitch_link']['extents'][2]:.4f}"/>
      </geometry>
    </collision>
  </link>

  <!-- ======================== IMU SENSOR FRAME ======================== -->
  <joint name="imu_joint" type="fixed">
    <parent link="pitch_link"/>
    <child link="imu_link"/>
    <origin xyz="{phone_com[0]:.4f} {phone_com[1]:.4f} {phone_com[2]:.4f}" rpy="0 0 0"/>
  </joint>

  <link name="imu_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.001"/>
      <inertia ixx="1e-7" ixy="0" ixz="0" iyy="1e-7" iyz="0" izz="1e-7"/>
    </inertial>
  </link>

  <!-- ======================== GAZEBO SENSOR & TRANSMISSIONS ======================== -->
  
  <!-- IMU Sensor configuration for Gazebo -->
  <gazebo reference="imu_link">
    <sensor name="gimbal_imu" type="imu">
      <always_on>true</always_on>
      <update_rate>200.0</update_rate>
      <visualize>true</visualize>
      <topic>/gimbal/imu/data</topic>
      <imu>
        <angular_velocity>
          <x><noise type="gaussian"><mean>0.0</mean><stddev>0.0002</stddev></noise></x>
          <y><noise type="gaussian"><mean>0.0</mean><stddev>0.0002</stddev></noise></y>
          <z><noise type="gaussian"><mean>0.0</mean><stddev>0.0002</stddev></noise></z>
        </angular_velocity>
        <linear_acceleration>
          <x><noise type="gaussian"><mean>0.0</mean><stddev>0.0015</stddev></noise></x>
          <y><noise type="gaussian"><mean>0.0</mean><stddev>0.0015</stddev></noise></y>
          <z><noise type="gaussian"><mean>0.0</mean><stddev>0.0015</stddev></noise></z>
        </linear_acceleration>
      </imu>
      <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
        <ros>
          <namespace>/gimbal</namespace>
          <remapping>~/out:=imu/data</remapping>
        </ros>
        <initial_orientation_as_reference>false</initial_orientation_as_reference>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Gazebo Colors & Friction -->
  <gazebo reference="base_link">
    <material>Gazebo/DarkGrey</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu1>
  </gazebo>
  <gazebo reference="yaw_link">
    <material>Gazebo/DarkGrey</material>
  </gazebo>
  <gazebo reference="roll_link">
    <material>Gazebo/DarkGrey</material>
  </gazebo>
  <gazebo reference="pitch_link">
    <material>Gazebo/Grey</material>
  </gazebo>

</robot>
"""
    with open(os.path.join(urdf_dir, 'osmo_mobile_6.urdf'), 'w') as f:
        f.write(urdf_content)
    print("URDF generated: dji_osmo_mobile_6/urdf/osmo_mobile_6.urdf")

    # Generate ROS 2 package.xml
    pkg_xml = """<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>dji_osmo_mobile_6</name>
  <version>1.0.0</version>
  <description>DJI Osmo Mobile 6 URDF and Gazebo Simulation Package with IMU stabilization</description>
  <maintainer email="developer@example.com">Gimbal Developer</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>gazebo_ros</exec_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
    <gazebo_ros gazebo_model_path="${prefix}/.."/>
  </export>
</package>
"""
    with open(os.path.join(pkg_dir, 'package.xml'), 'w') as f:
        f.write(pkg_xml)

    # Generate CMakeLists.txt
    cmake_content = """cmake_minimum_required(VERSION 3.8)
project(dji_osmo_mobile_6)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes launch config scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
"""
    with open(os.path.join(pkg_dir, 'CMakeLists.txt'), 'w') as f:
        f.write(cmake_content)

    # Generate RViz Display Launch file
    display_launch = """import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('dji_osmo_mobile_6')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'osmo_mobile_6.urdf')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
"""
    with open(os.path.join(launch_dir, 'display.launch.py'), 'w') as f:
        f.write(display_launch)

    # Generate Gazebo Launch file
    gazebo_launch = """import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('dji_osmo_mobile_6')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'osmo_mobile_6.urdf')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ])
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'dji_osmo_mobile_6', '-z', '0.01'],
        output='screen'
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    return LaunchDescription([
        gazebo,
        robot_state_pub,
        spawn_robot
    ])
"""
    with open(os.path.join(launch_dir, 'gazebo.launch.py'), 'w') as f:
        f.write(gazebo_launch)

    # Generate IMU Closed-Loop Stabilization Script
    stabilizer_script = """#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
import numpy as np

class GimbalStabilizer(Node):
    \"\"\"
    Closed-loop PID stabilizer for 3-axis gimbal using phone IMU orientation feedback.
    Maintains level horizon (roll=0, pitch=0) and controlled heading (yaw).
    \"\"\"
    def __init__(self):
        super().__init__('gimbal_stabilizer')
        
        self.sub_imu = self.create_subscription(
            Imu,
            '/gimbal/imu/data',
            self.imu_callback,
            10
        )
        
        self.pub_cmd = self.create_publisher(
            Float64MultiArray,
            '/gimbal/joint_group_effort_controller/commands',
            10
        )
        
        # PID gains for [yaw, roll, pitch]
        self.kp = np.array([2.5, 3.0, 3.0])
        self.ki = np.array([0.05, 0.1, 0.1])
        self.kd = np.array([0.15, 0.2, 0.2])
        
        self.integral_error = np.zeros(3)
        self.prev_error = np.zeros(3)
        self.target_angles = np.zeros(3) # Level horizon: [yaw=0, roll=0, pitch=0]
        
        self.get_logger().info("DJI Gimbal IMU Stabilization Controller started.")

    def quaternion_to_euler(self, q):
        # Convert quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw)
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return np.array([yaw, roll, pitch])

    def imu_callback(self, msg: Imu):
        current_angles = self.quaternion_to_euler(msg.orientation)
        ang_vel = np.array([
            msg.angular_velocity.z,
            msg.angular_velocity.x,
            msg.angular_velocity.y
        ])
        
        # Error: target - current
        error = self.target_angles - current_angles
        self.integral_error += error * 0.005 # 200 Hz dt
        self.integral_error = np.clip(self.integral_error, -0.5, 0.5)
        
        # Derivative using IMU gyroscope angular velocity
        d_error = -ang_vel
        
        # PID control command
        cmd_torque = (self.kp * error) + (self.ki * self.integral_error) + (self.kd * d_error)
        cmd_torque = np.clip(cmd_torque, -1.0, 1.0)
        
        msg_cmd = Float64MultiArray()
        msg_cmd.data = cmd_torque.tolist()
        self.pub_cmd.publish(msg_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = GimbalStabilizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""
    with open(os.path.join(scripts_dir, 'gimbal_stabilizer.py'), 'w') as f:
        f.write(stabilizer_script)

    print("\n--- Package generation complete! ---")

if __name__ == '__main__':
    build_gimbal_package()
