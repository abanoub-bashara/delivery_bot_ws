#!/usr/bin/env python3
# delivery_bot_control/src/odometry_node.py
import math, numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdometryNode(Node):
    def __init__(self):
        super().__init__('omni_odometry')

        # --- Params (must match your kinematics/controller) ---
        self.declare_parameter('wheel_radius', 0.024)
        self.declare_parameter('robot_radius', 0.1155)
        self.declare_parameter('wheel_angles_deg', [30.0, 150.0, 270.0])
        self.declare_parameter('joint_names', ['wheel1_joint','wheel2_joint','wheel3_joint'])
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('rate', 50.0)  # only used for dt fallback
        self.declare_parameter('publish_tf', False)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)


        self.r = float(self.get_parameter('wheel_radius').value)
        self.L = float(self.get_parameter('robot_radius').value)
        self.joint_names = list(self.get_parameter('joint_names').value)
        thetas = np.deg2rad(np.array(self.get_parameter('wheel_angles_deg').value, dtype=float))
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.dt_fallback = 1.0/float(self.get_parameter('rate').value)

        # v_rim = M @ [vx, vy, wz]  =>  [vx, vy, wz] = invM @ v_rim
        M = np.vstack([[-np.sin(t), np.cos(t), self.L] for t in thetas])
        self.invM = np.linalg.inv(M)

        # State
        self.x = 0.0; self.y = 0.0; self.theta = 0.0
        self.last_stamp: Time | None = None
        self.last_pos = {n: None for n in self.joint_names}

        # IO
        self.sub = self.create_subscription(JointState, '/joint_states', self._on_js, 50)
        self.odom_pub  = self.create_publisher(Odometry, '/wheel_odom', 50)
        self.tf_br     = TransformBroadcaster(self)

        self.get_logger().info('Omni odometry ready.')

    def _on_js(self, msg: JointState):
        # --- dt from message stamp (sim-time friendly) ---
        now = Time(seconds=msg.header.stamp.sec, nanoseconds=msg.header.stamp.nanosec,
                   clock_type=self.get_clock().clock_type)
        if self.last_stamp is None:
            self.last_stamp = now
            return
        dt = (now - self.last_stamp).nanoseconds * 1e-9
        self.last_stamp = now
        if not (0.0 < dt < 0.2):
            dt = self.dt_fallback

        # --- Build wheel omega (rad/s) with robust fallback ---
        name_to_idx = {n:i for i,n in enumerate(msg.name)}
        omega = []
        for jn in self.joint_names:
            if jn not in name_to_idx:
                omega.append(0.0); continue
            i = name_to_idx[jn]

            vel_ok = (len(msg.velocity) > i and msg.velocity[i] == msg.velocity[i])  # not NaN
            if vel_ok:
                omega.append(float(msg.velocity[i]))
            else:
                # finite-difference position if velocity missing/NaN
                pos_ok = (len(msg.position) > i)
                if pos_ok and self.last_pos[jn] is not None and dt > 0.0:
                    omega.append((float(msg.position[i]) - self.last_pos[jn]) / dt)
                else:
                    omega.append(0.0)

            # remember last position for next diff
            if len(msg.position) > i:
                self.last_pos[jn] = float(msg.position[i])

        omega = np.array(omega, dtype=float)

        # --- Invert kinematics: body twist in base_link ---
        v_body = self.invM.dot(self.r * omega)
        vx, vy, wz = float(v_body[0]), float(v_body[1]), float(v_body[2])

        # --- Integrate to odom (world) ---
        c, s = math.cos(self.theta), math.sin(self.theta)
        self.x     += ( vx*c - vy*s ) * dt
        self.y     += ( vx*s + vy*c ) * dt
        self.theta += wz * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))  # normalize

        # --- Publish odometry ---
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id  = self.child_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        qz, qw = math.sin(self.theta/2.0), math.cos(self.theta/2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x  = vx
        odom.twist.twist.linear.y  = vy
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

        # --- TF odom -> base_link ---
        if self.publish_tf:
            t = TransformStamped()
            t.header = odom.header
            t.child_frame_id = self.child_frame_id
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_br.sendTransform(t)

def main():
    rclpy.init()
    rclpy.spin(OdometryNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
