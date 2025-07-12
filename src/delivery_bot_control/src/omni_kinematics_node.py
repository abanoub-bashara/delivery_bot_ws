# omni_kinematics_node.py
# Description:
#   Subscribe to /cmd_vel (Twist) and compute 3-wheel velocities for a 120° omni layout,
#   then publish a trajectory_msgs/JointTrajectory on /omni_controller/joint_trajectory.
#
# Outline:
#   import rclpy
#   from rclpy.node import Node
#   from geometry_msgs.msg import Twist
#   from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
#   import numpy as np
#
#   class OmniKinematicsNode(Node):
#       def __init__(self):
#           super().__init__('omni_kinematics')
#           # params: wheel_radius, chassis_radius
#           # pub + sub
#       def cmd_vel_callback(self, msg: Twist):
#           # compute wheel speeds
#           # fill JointTrajectory and publish
#   def main():
#       rclpy.init()
#       node = OmniKinematicsNode()
#       rclpy.spin(node)
#       …