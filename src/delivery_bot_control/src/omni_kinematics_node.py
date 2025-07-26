#!/usr/bin/env python3
# omni_kinematics_node.py
# Description:
#   Subscribe to /cmd_vel (Twist) and compute 3-wheel velocities for a 120° omni layout,
#   then publish a trajectory_msgs/JointTrajectory on /omni_controller/joint_trajectory.
#
# Outline:

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

#this node basically listens to commands for desired linear and angular velocities
#and computes the required wheel velocities for a 3-wheel omni-directional robot

class OmniKinematicsNode(Node):
    def __init__(self):
        super().__init__('omni_kinematics')
        self.declare_parameter('wheel_radius', 0.1) #CHANGE VALUE
        self.declare_parameter('wheel_distance', 0.5) #CHANGE VALUE
        L = self.get_parameter('wheel_distance').value
        r = self.get_parameter('wheel_radius').value

        thetas = np.deg2rad(np.array([0, 120, 240]))
        self.M = np.vstack([[-np.sin(t), np.cos(t), L] for t in thetas])
        #thetas uses deg2rad to convert degrees to radians, holds 120° angle layout
        #self.M is the transformation matrix for wheel velocities

        #subscribe to cmd vel whih carries linear and angular velocities in Twist messages
        #with a queue size of 10, and whenever a new message is received,
        # the cmd_vel_callback is called
        self.cmd_sub = self.create_subscription(
            Twist, 
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

       #configure qos profile for the publisher

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        #create a publisher for JointTrajectory messages
        #on the topic /omni_controller/joint_trajectory with the defined QoS profile
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/omni_controller/joint_trajectory',
            qos
        )

        self.joint_names = ['wheel1', 'wheel2', 'wheel3']

    def cmd_vel_callback(self, msg: Twist):
        #unpack the tist 
        vx, vy, omega = msg.linear.x, msg.linear.y, msg.angular.z

        
        v_rim = self.M.dot([vx, vy, omega])
        omega_wheels = (v_rim/self.wheel_radius).tolist()

        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.velocities = omega_wheels
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 500_000_000

        traj.points = [point]
        self.traj_pub.publish(traj)

def main():
    #initalize the ROS 2 Python client library
    rclpy.init()
    #create an instance of the OmniKinematicsNode
    node = OmniKinematicsNode()
    #spin the node to keep it active and processing callbacks i.e., 
    #subscribing to cmd_vel and publishing joint trajectories
    rclpy.spin(node)
    #clean up and shutdown
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    