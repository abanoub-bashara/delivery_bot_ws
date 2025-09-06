#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray

class OmniVelocitySmoother(Node):
    def __init__(self):
        super().__init__('omni_velocity_smoother')
        
        # Parameters
        self.declare_parameter('max_linear_accel', 2.0)    # m/s²
        self.declare_parameter('max_angular_accel', 3.0)   # rad/s²
        self.declare_parameter('max_wheel_accel', 10.0)    # rad/s² per wheel
        self.declare_parameter('update_rate', 50.0)        # Hz
        self.declare_parameter('robot_radius', 0.115)
        self.declare_parameter('wheel_radius', 0.024)
        self.declare_parameter('wheel_angles_deg', [30, 150, 270])
        
        # Get parameters
        self.max_linear_accel = self.get_parameter('max_linear_accel').value
        self.max_angular_accel = self.get_parameter('max_angular_accel').value  
        self.max_wheel_accel = self.get_parameter('max_wheel_accel').value
        self.update_rate = self.get_parameter('update_rate').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_angles_deg = self.get_parameter('wheel_angles_deg').value
        
        self.dt = 1.0 / self.update_rate
        
        # State variables
        self.current_twist = [0.0, 0.0, 0.0]  # [vx, vy, wz]
        self.target_twist = [0.0, 0.0, 0.0]
        self.current_wheel_vels = [0.0, 0.0, 0.0]
        
        # Publishers and subscribers
        self.cmd_sub = self.create_subscription(
            TwistStamped, '/cmd_vel_raw', self.cmd_callback, 10)
        self.cmd_pub = self.create_publisher(
            TwistStamped, '/omni_drive_controller/cmd_vel', 10)
        self.wheel_pub = self.create_publisher(
            Float64MultiArray, '/debug/wheel_velocities', 10)
        
        # Timer for smooth updates
        self.timer = self.create_timer(self.dt, self.update_velocities)
        
        self.get_logger().info('Omni Velocity Smoother started')
        self.get_logger().info(f'Max linear accel: {self.max_linear_accel} m/s²')
        self.get_logger().info(f'Max angular accel: {self.max_angular_accel} rad/s²')
        self.get_logger().info(f'Max wheel accel: {self.max_wheel_accel} rad/s²')
        self.get_logger().info(f'Update rate: {self.update_rate} Hz (dt = {self.dt:.4f}s)')
        
    def cmd_callback(self, msg):
        """Receive target velocity command"""
        self.target_twist = [
            msg.twist.linear.x,
            msg.twist.linear.y, 
            msg.twist.angular.z
        ]
        
    def omni_forward_kinematics(self, vx, vy, wz):
        """Convert body twist to wheel velocities (controller's formula)"""
        wheel_vels = []
        for angle_deg in self.wheel_angles_deg:
            angle_rad = math.radians(angle_deg)
            wheel_vel = (
                math.sin(angle_rad) * vx -
                math.cos(angle_rad) * vy -
                self.robot_radius * wz
            ) / self.wheel_radius
            wheel_vels.append(wheel_vel)
        return wheel_vels
        
    def limit_acceleration(self, current, target, max_accel, dt):
        """Limit acceleration between current and target values"""
        diff = target - current
        max_change = max_accel * dt
        
        if abs(diff) <= max_change:
            return target
        else:
            return current + math.copysign(max_change, diff)
            
    def coordinated_wheel_limiting(self, target_wheel_vels):
        """
        Apply acceleration limits while maintaining wheel velocity ratios
        This prevents unwanted yaw during acceleration
        Uses fixed dt for consistency
        """
        # Calculate required accelerations for each wheel using fixed dt
        wheel_accels = []
        for i in range(len(target_wheel_vels)):
            accel = abs((target_wheel_vels[i] - self.current_wheel_vels[i]) / self.dt)
            wheel_accels.append(accel)
            
        # Find the wheel that would exceed limits the most
        max_accel_ratio = max(wheel_accels) / self.max_wheel_accel if max(wheel_accels) > 0 else 0
        
        if max_accel_ratio <= 1.0:
            # All wheels can reach target without exceeding limits
            self.current_wheel_vels = target_wheel_vels[:]
        else:
            # Scale down all accelerations proportionally to maintain ratios
            scale_factor = 1.0 / max_accel_ratio
            for i in range(len(target_wheel_vels)):
                vel_change = (target_wheel_vels[i] - self.current_wheel_vels[i]) * scale_factor
                self.current_wheel_vels[i] += vel_change
                
        return self.current_wheel_vels[:]
        
    def update_velocities(self):
        """Main update loop with coordinated acceleration limiting"""
        # Method 1: Limit at body velocity level (simpler)
        smooth_vx = self.limit_acceleration(
            self.current_twist[0], self.target_twist[0], 
            self.max_linear_accel, self.dt)
        smooth_vy = self.limit_acceleration(
            self.current_twist[1], self.target_twist[1], 
            self.max_linear_accel, self.dt)
        smooth_wz = self.limit_acceleration(
            self.current_twist[2], self.target_twist[2], 
            self.max_angular_accel, self.dt)
            
        self.current_twist = [smooth_vx, smooth_vy, smooth_wz]
        
        # Method 2: Also apply wheel-level coordinated limiting
        target_wheel_vels = self.omni_forward_kinematics(
            smooth_vx, smooth_vy, smooth_wz)
        final_wheel_vels = self.coordinated_wheel_limiting(target_wheel_vels)
        
        # Publish smoothed twist command
        cmd_msg = TwistStamped()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.header.frame_id = 'base_link'
        cmd_msg.twist.linear.x = smooth_vx
        cmd_msg.twist.linear.y = smooth_vy
        cmd_msg.twist.angular.z = smooth_wz
        self.cmd_pub.publish(cmd_msg)
        
        # Publish debug wheel velocities
        wheel_msg = Float64MultiArray()
        wheel_msg.data = final_wheel_vels
        self.wheel_pub.publish(wheel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OmniVelocitySmoother()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()