#!/usr/bin/env python3
# delivery_bot_control/src/omni_kinematics_node.py
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class OmniKinematicsNode(Node):
    def __init__(self):
        super().__init__('omni_kinematics')

        # ---- Parameters (tune to your robot) ----
        self.declare_parameter('wheel_radius', 0.024)        # m
        self.declare_parameter('robot_radius', 0.1155)       # m (center -> wheel)
        self.declare_parameter('wheel_angles_deg', [0.0, 120.0, 240.0])  # wheel placement angles
        self.declare_parameter('cmd_rate', 50.0)             # Hz publish rate to controller
        self.declare_parameter('cmd_timeout', 0.5)           # s: stop if no cmd_vel for this long

        # Load params
        self.r = float(self.get_parameter('wheel_radius').value)
        self.L = float(self.get_parameter('robot_radius').value)
        angles_deg = list(self.get_parameter('wheel_angles_deg').value)
        self.phi = np.deg2rad(np.array(angles_deg, dtype=float))
        self.cmd_dt = 1.0 / float(self.get_parameter('cmd_rate').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        # Build forward kinematics matrix:
        # [v1, v2, v3]^T = M @ [vx, vy, wz]^T, where vi are rim linear velocities
        self.M = np.vstack([[-np.sin(t), np.cos(t), self.L] for t in self.phi])

        # I/O
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.pub = self.create_publisher(Float64MultiArray,
                                         '/omni_wheel_drive_controller/commands', 10)

        # State
        self.last_cmd = Twist()
        self.last_cmd_time = self.get_clock().now().nanoseconds * 1e-9

        # Periodic publisher so the robot keeps moving smoothly
        self.timer = self.create_timer(self.cmd_dt, self._tick)

        self.get_logger().info(
            f'Omni kinematics ready: r={self.r:.4f} m, L={self.L:.4f} m, '
            f'angles(deg)={angles_deg}, rate={1.0/self.cmd_dt:.0f} Hz'
        )

    def _on_cmd_vel(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now().nanoseconds * 1e-9

    def _tick(self):
        # Timeout safety: zero command if stale
        now = self.get_clock().now().nanoseconds * 1e-9
        if (now - self.last_cmd_time) > self.cmd_timeout:
            vx = vy = wz = 0.0
        else:
            vx = float(self.last_cmd.linear.x)
            vy = float(self.last_cmd.linear.y)
            wz = float(self.last_cmd.angular.z)

        # Convert body twist -> wheel angular speeds
        # rim linear speeds:
        v_rim = self.M.dot(np.array([vx, vy, wz], dtype=float))
        # wheel angular speeds (rad/s):
        omega_wheels = (v_rim / self.r).astype(float)

        self.pub.publish(Float64MultiArray(data=omega_wheels.tolist()))

def main():
    rclpy.init()
    node = OmniKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
