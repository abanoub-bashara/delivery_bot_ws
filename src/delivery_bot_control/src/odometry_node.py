#!/usr/bin/env python3
# delivery_bot_control/src/odometry_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import math

class OdometryNode(Node):
    def __init__(self):
        super().__init__('omni_odometry')

        # radius of each wheel
        self.declare_parameter('wheel_radius', 0.024)
        # distance from robot center to each wheel
        self.declare_parameter('robot_radius', 0.1155)
        r = self.get_parameter('wheel_radius').value
        L = self.get_parameter('robot_radius').value

        # build the inverse kinematics matrix
        thetas = np.deg2rad([0, 120, 240])
        M = np.vstack([[-np.sin(t), np.cos(t), L] for t in thetas])
        self.M_inv = np.linalg.inv(M)

        # initial pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now().nanoseconds * 1e-9

        # subscribers / publishers
        self.joint_names = ['wheel1_joint', 'wheel2_joint', 'wheel3_joint']
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10)
        self.odom_pub  = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.r = r

    def joint_states_callback(self, msg: JointState):
        # compute time delta
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = now - self.last_time
        self.last_time = now

        # gather wheel angular velocities
        omega_wheels = np.array([msg.velocity[msg.name.index(n)] for n in self.joint_names])

        # convert to rim velocities
        v_rim = omega_wheels * self.r
        vx, vy, omega = self.M_inv.dot(v_rim)

        # integrate pose
        dx = vx * dt; dy = vy * dt; dtheta = omega * dt
        self.x += dx * math.cos(self.theta) - dy * math.sin(self.theta)
        self.y += dx * math.sin(self.theta) + dy * math.cos(self.theta)
        self.theta += dtheta

        # publish Odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        qz = math.sin(self.theta/2);
        qw = math.cos(self.theta/2)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x  = vx
        odom.twist.twist.linear.y  = vy
        odom.twist.twist.angular.z = omega
        self.odom_pub.publish(odom)

        # broadcast TF
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z    = qz
        t.transform.rotation.w    = qw
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = OdometryNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
