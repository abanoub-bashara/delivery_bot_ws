#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class OmniKinematicsNode(Node):
    def __init__(self):
        super().__init__('omni_kinematics')

        # ---- Parameters ----
        self.declare_parameter('wheel_radius', 0.024)          # m
        self.declare_parameter('robot_radius', 0.1155)         # m
        self.declare_parameter('wheel_angles_deg', [0.0, 120.0, 240.0])
        self.declare_parameter('cmd_rate', 50.0)               # Hz (publish rate)
        self.declare_parameter('cmd_timeout', 0.5)             # s (stop if stale)
        # Ramp & limits
        self.declare_parameter('max_wheel_accel', 20.0)        # rad/s^2 (per wheel)
        self.declare_parameter('max_wheel_speed', 100.0)       # rad/s clamp (safety)
        self.declare_parameter('max_wheel_jerk', 0.0)          # rad/s^3 (0 = disabled)

        # Load params
        self.r = float(self.get_parameter('wheel_radius').value)
        self.L = float(self.get_parameter('robot_radius').value)
        angles_deg = list(self.get_parameter('wheel_angles_deg').value)
        phi = np.deg2rad(np.array(angles_deg, dtype=float))
        self.dt = 1.0 / float(self.get_parameter('cmd_rate').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.max_acc = float(self.get_parameter('max_wheel_accel').value)
        self.max_w = float(self.get_parameter('max_wheel_speed').value)
        self.max_jerk = float(self.get_parameter('max_wheel_jerk').value)

        # v_rim = M @ [vx, vy, wz]
        self.M = np.vstack([[-np.sin(t), np.cos(t), self.L] for t in phi])

        # I/O
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.pub = self.create_publisher(Float64MultiArray,
                                         '/omni_wheel_drive_controller/commands', 10)

        # State
        self.last_cmd = Twist()
        self.last_cmd_time = self.get_clock().now().nanoseconds * 1e-9
        self.prev_w = np.zeros(3)        # previous wheel speeds (rad/s)
        self.prev_alpha = np.zeros(3)    # previous wheel accelerations (rad/s^2) for jerk limiting

        # Periodic publisher
        self.timer = self.create_timer(self.dt, self._tick)

        self.get_logger().info(
            f'Omni kinematics ready: r={self.r:.4f} m, L={self.L:.4f} m, '
            f'angles(deg)={angles_deg}, rate={1.0/self.dt:.0f} Hz'
        )

    def _on_cmd_vel(self, msg: Twist):
        # Just store the latest command and time; ramping/publish happens in _tick.
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now().nanoseconds * 1e-9

    def _target_wheels(self, vx: float, vy: float, wz: float) -> np.ndarray:
        v_rim = self.M.dot(np.array([vx, vy, wz], dtype=float))
        return v_rim / self.r  # rad/s for each wheel

    def _tick(self):
        # Timeout safety
        now = self.get_clock().now().nanoseconds * 1e-9
        if (now - self.last_cmd_time) > self.cmd_timeout:
            vx = vy = wz = 0.0
        else:
            vx = float(self.last_cmd.linear.x)
            vy = float(self.last_cmd.linear.y)
            wz = float(self.last_cmd.angular.z)

        w_tgt = self._target_wheels(vx, vy, wz)

        # ---- Accel / jerk limiting ramp ----
        if self.max_jerk > 0.0:
            # Compute desired acceleration to reach target in one step
            alpha_des = (w_tgt - self.prev_w) / self.dt
            # Limit jerk: change in acceleration per step
            delta_alpha = np.clip(alpha_des - self.prev_alpha,
                                  -self.max_jerk * self.dt, self.max_jerk * self.dt)
            alpha = self.prev_alpha + delta_alpha
            # Also cap absolute acceleration
            alpha = np.clip(alpha, -self.max_acc, self.max_acc)
            w_cmd = self.prev_w + alpha * self.dt
            self.prev_alpha = alpha
        else:
            # Simple accel limit: cap delta wheel speed per step
            delta_w = np.clip(w_tgt - self.prev_w, -self.max_acc * self.dt, self.max_acc * self.dt)
            w_cmd = self.prev_w + delta_w

        # Clamp to max wheel speed for safety
        w_cmd = np.clip(w_cmd, -self.max_w, self.max_w)

        # Publish & update state
        self.pub.publish(Float64MultiArray(data=w_cmd.astype(float).tolist()))
        self.prev_w = w_cmd

def main():
    rclpy.init()
    node = OmniKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
