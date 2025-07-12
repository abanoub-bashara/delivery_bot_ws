# odometry_node.py
# Description:
#   Listen to /joint_states, integrate wheel displacements into a robot pose,
#   and publish nav_msgs/Odometry on /odom.
#
# Outline:
#   import rclpy
#   from rclpy.node import Node
#   from sensor_msgs.msg import JointState
#   from nav_msgs.msg import Odometry
#   import math
#
#   class OdometryNode(Node):
#       def __init__(self):
#           super().__init__('odometry_node')
#           # params: wheel_radius, baseline_distance
#           # state: x, y, theta
#           # pub + sub
#       def joint_state_callback(self, msg: JointState):
#           # compute Δdistances, update x/y/theta
#           # fill and publish Odometry
#   def main():
#       rclpy.init()
#       node = OdometryNode()
#       …
