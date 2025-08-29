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
        
        # Subscriber: Listen to Nav2's Twist commands only
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )
        
        # Track if we've received any messages
        self.last_msg_time = None
        
        self.get_logger().info('✅ TwistToStamped node started — bridging /cmd_vel → /omni_drive_controller/cmd_vel')
        self.get_logger().info('📌 Teleop publishes directly to /omni_drive_controller/cmd_vel (stamped), Nav2 goes through this bridge')
    
    def listener_callback(self, msg: Twist):
        # Check if this is a zero message (could be from teleop cleanup)
        is_zero_twist = (msg.linear.x == 0.0 and msg.linear.y == 0.0 and 
                        msg.linear.z == 0.0 and msg.angular.x == 0.0 and 
                        msg.angular.y == 0.0 and msg.angular.z == 0.0)
        
        # Only convert and publish non-zero messages or if it's been a while since last message
        current_time = self.get_clock().now()
        if not is_zero_twist or self.last_msg_time is None or (current_time - self.last_msg_time).nanoseconds > 1e9:
            stamped_msg = TwistStamped()
            stamped_msg.header.stamp = current_time.to_msg()
            stamped_msg.header.frame_id = 'base_link'
            stamped_msg.twist = msg
            self.publisher_.publish(stamped_msg)
            self.last_msg_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = TwistToStamped()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()