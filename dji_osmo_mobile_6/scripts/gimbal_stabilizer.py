#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
import numpy as np

class GimbalStabilizer(Node):
    """
    Closed-loop PID stabilizer for 3-axis gimbal using phone IMU orientation feedback.
    Maintains level horizon (roll=0, pitch=0) and controlled heading (yaw).
    """
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
