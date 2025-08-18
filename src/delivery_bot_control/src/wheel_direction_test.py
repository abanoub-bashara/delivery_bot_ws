#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import time

class WheelDirectionTest(Node):
    def __init__(self):
        super().__init__('wheel_direction_test')
        
        # Robot parameters
        self.r = 0.024    # wheel radius
        self.L = 0.1155   # robot radius
        
        # Wheel angles (confirmed correct since wheel1 doesn't move for pure X)
        phi = np.deg2rad([270.0, 30.0, 150.0])
        
        # Kinematic matrix
        self.M = np.array([
            [-np.sin(phi[0]), np.cos(phi[0]), self.L],  # Wheel 1 (0°)
            [-np.sin(phi[1]), np.cos(phi[1]), self.L],  # Wheel 2 (120°)  
            [-np.sin(phi[2]), np.cos(phi[2]), self.L]   # Wheel 3 (240°)
        ])
        
        # Publisher
        self.pub = self.create_publisher(Float64MultiArray,
                                         '/omni_wheel_drive_controller/commands', 10)
        
        # Print the kinematic matrix for reference
        self.get_logger().info("=" * 60)
        self.get_logger().info("OMNI WHEEL DIRECTION TEST - AUTOMATIC MODE")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Kinematic Matrix (each row = wheel):")
        for i, row in enumerate(self.M):
            self.get_logger().info(f"  Wheel {i+1}: [-sin({phi[i]*180/np.pi:5.1f}°), cos({phi[i]*180/np.pi:5.1f}°), L] = [{row[0]:6.3f}, {row[1]:6.3f}, {row[2]:6.3f}]")
        
        # Expected wheel speeds for test movements
        self.get_logger().info("\nExpected wheel speeds for reference:")
        test_vx = self._target_wheels(0.1, 0.0, 0.0)
        test_vy = self._target_wheels(0.0, 0.1, 0.0)
        test_wz = self._target_wheels(0.0, 0.0, 0.1)
        
        self.get_logger().info(f"Pure X (0.1 m/s): [{test_vx[0]:6.2f}, {test_vx[1]:6.2f}, {test_vx[2]:6.2f}] rad/s")
        self.get_logger().info(f"Pure Y (0.1 m/s): [{test_vy[0]:6.2f}, {test_vy[1]:6.2f}, {test_vy[2]:6.2f}] rad/s") 
        self.get_logger().info(f"Pure Z (0.1 rad/s): [{test_wz[0]:6.2f}, {test_wz[1]:6.2f}, {test_wz[2]:6.2f}] rad/s")
        
        # Test sequence
        self.tests = [
            {"name": "WHEEL1 INDIVIDUAL SPIN", "wheels": [15.0, 0.0, 0.0], "description": "Only wheel1 should spin"},
            {"name": "WHEEL2 INDIVIDUAL SPIN", "wheels": [0.0, 15.0, 0.0], "description": "Only wheel2 should spin"},
            {"name": "WHEEL3 INDIVIDUAL SPIN", "wheels": [0.0, 0.0, 15.0], "description": "Only wheel3 should spin"},
            {"name": "PURE X MOVEMENT", "cmd": [0.15, 0.0, 0.0], "description": "Robot should move in +X direction"},
            {"name": "PURE Y MOVEMENT", "cmd": [0.0, 0.15, 0.0], "description": "Robot should move 90° left of +X direction"},
            {"name": "PURE ROTATION", "cmd": [0.0, 0.0, 0.15], "description": "Robot should rotate counterclockwise"},
            {"name": "DIAGONAL MOVEMENT", "cmd": [0.1, 0.1, 0.0], "description": "Robot should move at 45° angle"},
            {"name": "STOP", "wheels": [0.0, 0.0, 0.0], "description": "Robot should stop"}
        ]
        
        # Start countdown
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("STARTING TEST SEQUENCE IN 5 SECONDS...")
        self.get_logger().info("Each test runs for 8 seconds with 2 second pause between")
        self.get_logger().info("Watch your robot carefully and note the movement patterns!")
        self.get_logger().info("=" * 60)
        
        self.test_step = 0
        self.countdown = 5
        self.in_test = False
        self.timer = self.create_timer(1.0, self._run_sequence)  # 1 second intervals

    def _target_wheels(self, vx: float, vy: float, wz: float) -> np.ndarray:
        """Convert robot velocities to wheel speeds"""
        robot_vel = np.array([vx, vy, wz], dtype=float)
        v_rim = self.M @ robot_vel
        return v_rim / self.r

    def _publish_wheel_speeds(self, wheel_speeds: np.ndarray, description: str):
        """Publish wheel speeds with description"""
        msg = Float64MultiArray()
        msg.data = wheel_speeds.astype(float).tolist()
        self.pub.publish(msg)
        self.get_logger().info(f"{description}: [{wheel_speeds[0]:6.2f}, {wheel_speeds[1]:6.2f}, {wheel_speeds[2]:6.2f}] rad/s")

    def _run_sequence(self):
        """Run the test sequence with timing"""
        
        # Initial countdown
        if self.countdown > 0:
            self.get_logger().info(f"Starting in {self.countdown} seconds...")
            self.countdown -= 1
            return
        
        # Check if we're done with all tests
        if self.test_step >= len(self.tests):
            self._publish_wheel_speeds(np.zeros(3), "FINAL STOP")
            self.get_logger().info("\n" + "=" * 60)
            self.get_logger().info("ALL TESTS COMPLETE!")
            self.get_logger().info("=" * 60)
            self.get_logger().info("Now analyze what you observed:")
            self.get_logger().info("1. Did individual wheels (tests 1-3) spin in the same direction?")
            self.get_logger().info("2. Did Pure X movement go in your expected forward direction?")
            self.get_logger().info("3. Did Pure Y movement go 90° left of the X direction?")
            self.get_logger().info("4. Did rotation work correctly (counterclockwise)?")
            self.get_logger().info("5. Did diagonal movement go at ~45° angle?")
            self.get_logger().info("\nPress Ctrl+C to exit when ready.")
            self.timer.destroy()  # Stop the timer
            return
        
        # Get current test
        test = self.tests[self.test_step]
        
        if not self.in_test:
            # Starting a new test
            self.get_logger().info(f"\n🔵 TEST {self.test_step + 1}: {test['name']}")
            self.get_logger().info(f"   Expected: {test['description']}")
            
            # Calculate and publish wheel speeds
            if 'wheels' in test:
                # Direct wheel speeds
                wheel_speeds = np.array(test['wheels'])
            else:
                # Convert from cmd_vel
                vx, vy, wz = test['cmd']
                wheel_speeds = self._target_wheels(vx, vy, wz)
            
            self._publish_wheel_speeds(wheel_speeds, f"Running {test['name']}")
            self.in_test = True
            self.test_duration = 0
            
        else:
            # Continue current test
            self.test_duration += 1
            
            # Show progress dots
            if self.test_duration <= 8:
                dots = "." * (self.test_duration % 4)
                self.get_logger().info(f"   Running{dots:<3} ({self.test_duration}/8 seconds)")
            
            # End test after 8 seconds
            if self.test_duration >= 8:
                self.get_logger().info(f"   ✅ {test['name']} complete")
                
                # Stop the robot briefly between tests
                self._publish_wheel_speeds(np.zeros(3), "Pausing between tests")
                
                # Move to next test
                self.test_step += 1
                self.in_test = False
                
                # Add pause between tests (except for the last one)
                if self.test_step < len(self.tests):
                    self.get_logger().info("   ⏸️  2 second pause...")
                    # Add a small delay by skipping next tick
                    self.skip_next = True
                    return

def main():
    rclpy.init()
    node = WheelDirectionTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0]
        node.pub.publish(msg)
        node.get_logger().info("Robot stopped.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()