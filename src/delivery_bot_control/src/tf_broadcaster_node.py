# tf_broadcaster_node.py
# Description:
#   Subscribe to /odom and broadcast the odom→base_link transform via tf2_ros.TransformBroadcaster.
#
# Outline:
#   import rclpy
#   from rclpy.node import Node
#   from nav_msgs.msg import Odometry
#   from tf2_ros import TransformBroadcaster
#   from geometry_msgs.msg import TransformStamped
#
#   class TfBroadcasterNode(Node):
#       def __init__(self):
#           super().__init__('tf_broadcaster')
#           # create TransformBroadcaster
#           # subscriber to /odom
#       def odom_callback(self, msg: Odometry):
#           # fill TransformStamped and send transform
#   def main():
#       rclpy.init()
#       node = TfBroadcasterNode()
#       …
