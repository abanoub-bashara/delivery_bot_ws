#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import math

class OdometryDriftAnalyzer(Node):
    def __init__(self):
        super().__init__('odom_drift_analyzer')
        
        # Transform listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)
        
        # Data storage
        self.odom_poses = []
        self.amcl_poses = []
        self.drift_history = []
        
        # Debug counters
        self.odom_count = 0
        self.amcl_count = 0
        
        # Analysis timer - reduced to 2 seconds for faster feedback
        self.timer = self.create_timer(2.0, self.analyze_drift)
        
        self.get_logger().info("Odometry drift analyzer started")
        self.get_logger().info("Waiting for /odom and /amcl_pose topics...")

    def odom_callback(self, msg):
        self.odom_count += 1
        self.odom_poses.append({
            'time': self.get_clock().now().nanoseconds / 1e9,
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'yaw': self.quaternion_to_yaw(msg.pose.pose.orientation)
        })
        
        if self.odom_count % 10 == 0:  # Log every 10th message
            self.get_logger().info(f"Received {self.odom_count} odom messages")
        
        # Keep only recent data (last 60 seconds)
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.odom_poses = [p for p in self.odom_poses if current_time - p['time'] < 60.0]

    def amcl_callback(self, msg):
        self.amcl_count += 1
        self.amcl_poses.append({
            'time': self.get_clock().now().nanoseconds / 1e9,
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'yaw': self.quaternion_to_yaw(msg.pose.pose.orientation)
        })
        
        if self.amcl_count % 5 == 0:  # Log every 5th message
            self.get_logger().info(f"Received {self.amcl_count} AMCL messages")
        
        # Keep only recent data
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.amcl_poses = [p for p in self.amcl_poses if current_time - p['time'] < 60.0]

    def quaternion_to_yaw(self, q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def analyze_drift(self):
        self.get_logger().info(f"Analysis check: odom_poses={len(self.odom_poses)}, amcl_poses={len(self.amcl_poses)}")
        
        if len(self.odom_poses) < 5 or len(self.amcl_poses) < 5:
            self.get_logger().info(f"Not enough data yet. Need 5 of each, have odom:{len(self.odom_poses)}, amcl:{len(self.amcl_poses)}")
            return
        
        try:
            # Get current map->odom transform
            self.get_logger().info("Trying to get map->odom transform...")
            transform = self.tf_buffer.lookup_transform(
                'map', 'odom', rclpy.time.Time())
            
            map_odom_x = transform.transform.translation.x
            map_odom_y = transform.transform.translation.y
            map_odom_yaw = self.quaternion_to_yaw(transform.transform.rotation)
            
            self.get_logger().info(f"Transform found: x={map_odom_x:.3f}, y={map_odom_y:.3f}, yaw={math.degrees(map_odom_yaw):.1f}°")
            
            # Calculate drift magnitude
            drift_magnitude = math.sqrt(map_odom_x**2 + map_odom_y**2)
            drift_angle = abs(map_odom_yaw)
            
            # Store drift history
            self.drift_history.append({
                'time': self.get_clock().now().nanoseconds / 1e9,
                'position_drift': drift_magnitude,
                'angular_drift': drift_angle
            })
            
            # Keep only recent history
            current_time = self.get_clock().now().nanoseconds / 1e9
            self.drift_history = [d for d in self.drift_history if current_time - d['time'] < 300.0]
            
            # Analyze drift trends - reduced requirement to 3 samples
            if len(self.drift_history) >= 3:
                recent_pos_drift = [d['position_drift'] for d in self.drift_history[-3:]]
                recent_ang_drift = [d['angular_drift'] for d in self.drift_history[-3:]]
                
                avg_pos_drift = np.mean(recent_pos_drift)
                avg_ang_drift = np.mean(recent_ang_drift)
                pos_drift_rate = (recent_pos_drift[-1] - recent_pos_drift[0]) / 6.0  # over 6 seconds
                
                # Assessment
                self.assess_odometry_quality(avg_pos_drift, avg_ang_drift, pos_drift_rate)
            else:
                self.get_logger().info(f"Building drift history: {len(self.drift_history)}/3 samples")
                
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f'Could not get map->odom transform: {ex}')
            
            # Check what transforms are available
            self.get_logger().info("Available transforms:")
            try:
                frames = self.tf_buffer._buffer.all_frames_as_yaml()
                self.get_logger().info(f"TF frames: {frames}")
            except:
                pass

    def assess_odometry_quality(self, avg_pos_drift, avg_ang_drift, drift_rate):
        self.get_logger().info(f"=== Odometry Quality Assessment ===")
        self.get_logger().info(f"Average position drift: {avg_pos_drift:.3f}m")
        self.get_logger().info(f"Average angular drift: {math.degrees(avg_ang_drift):.1f}°")
        self.get_logger().info(f"Position drift rate: {drift_rate:.4f}m/s")
        
        # Quality assessment
        if avg_pos_drift < 0.1 and avg_ang_drift < 0.1 and abs(drift_rate) < 0.01:
            quality = "EXCELLENT"
            recommendation = "Use default AMCL alphas (0.2)"
        elif avg_pos_drift < 0.3 and avg_ang_drift < 0.2 and abs(drift_rate) < 0.02:
            quality = "GOOD"
            recommendation = "Use slightly increased AMCL alphas (0.3-0.4)"
        elif avg_pos_drift < 0.5 and avg_ang_drift < 0.3 and abs(drift_rate) < 0.05:
            quality = "FAIR"
            recommendation = "Use moderately increased AMCL alphas (0.5-0.8)"
        elif avg_pos_drift < 1.0 and avg_ang_drift < 0.5:
            quality = "POOR"
            recommendation = "Use high AMCL alphas (1.0-1.5), check wheel calibration"
        else:
            quality = "VERY POOR"
            recommendation = "Use very high AMCL alphas (2.0+), urgently fix odometry"
        
        self.get_logger().info(f"Odometry Quality: {quality}")
        self.get_logger().info(f"Recommendation: {recommendation}")
        self.get_logger().info("=====================================")

def main(args=None):
    rclpy.init(args=args)
    analyzer = OdometryDriftAnalyzer()
    try:
        rclpy.spin(analyzer)
    except KeyboardInterrupt:
        pass
    finally:
        analyzer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()