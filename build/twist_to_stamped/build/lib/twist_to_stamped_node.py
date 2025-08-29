#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistToStamped(Node):
    def __init__(self):
        super().__init__('twist_to_stamped')

        # Publisher: TwistStamped to the topic your driver expects
        self.publisher_ = self.create_publisher(
            TwistStamped,
            '/omni_drive_controller/cmd_vel',
            10
        )

        # Subscriber: Listen to Nav2's Twist commands
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )

        self.get_logger().info('✅ TwistToStamped node started — bridging /cmd_vel → /omni_drive_controller/cmd_vel')

    def listener_callback(self, msg: Twist):
        stamped_msg = TwistStamped()
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        stamped_msg.header.frame_id = 'base_link'  # or change if needed
        stamped_msg.twist = msg
        self.publisher_.publish(stamped_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistToStamped()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
