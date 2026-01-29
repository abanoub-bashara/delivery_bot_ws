#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import yaml
import os
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_srvs.srv import Empty
from delivery_bot_nav.srv import SaveLocation, GetLocation, ListLocations
import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class LocationManager(Node):
    def __init__(self):
        super().__init__('location_manager')
        
        # Callback group for concurrent service calls
        self.callback_group = ReentrantCallbackGroup()
        
        # Get parameters
        self.declare_parameter('locations_file', 
                             os.path.expanduser('~/ros2_ws/src/delivery_bot_nav/config/saved_locations.yaml'))
        self.yaml_file = self.get_parameter('locations_file').get_parameter_value().string_value
        
        # TF buffer for getting current robot pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Load existing locations
        self.locations = self.load_locations()
        
        # Services
        self.save_location_srv = self.create_service(
            SaveLocation, 'location_manager/save_location', 
            self.save_location_callback, callback_group=self.callback_group)
        
        self.get_location_srv = self.create_service(
            GetLocation, 'location_manager/get_location', 
            self.get_location_callback, callback_group=self.callback_group)
        
        self.list_locations_srv = self.create_service(
            ListLocations, 'location_manager/list_locations', 
            self.list_locations_callback, callback_group=self.callback_group)
        
        self.reload_locations_srv = self.create_service(
            Empty, 'location_manager/reload_locations', 
            self.reload_locations_callback, callback_group=self.callback_group)
        
        self.get_logger().info(f"Location Manager initialized with {len(self.locations)} saved locations")
        self.get_logger().info(f"Using locations file: {self.yaml_file}")
        
        # Print available locations
        if self.locations:
            self.get_logger().info("Available locations:")
            for name, data in self.locations.items():
                desc = data.get('description', 'No description')
                self.get_logger().info(f"  - {name}: {desc}")
        else:
            self.get_logger().info("No saved locations found. Use 'save location <n>' to create some!")
    
    def load_locations(self):
        """Load locations from YAML file"""
        try:
            if os.path.exists(self.yaml_file):
                with open(self.yaml_file, 'r') as file:
                    data = yaml.safe_load(file)
                    if data and 'saved_locations' in data:
                        return data['saved_locations']
                    else:
                        self.get_logger().warning(f"No 'saved_locations' found in {self.yaml_file}")
                        return {}
            else:
                self.get_logger().warning(f"Locations file {self.yaml_file} does not exist. Starting with empty locations.")
                return {}
        except Exception as e:
            self.get_logger().error(f"Error loading locations file: {e}")
            return {}
    
    def save_locations_to_file(self):
        """Save current locations to YAML file"""
        try:
            # Ensure directory exists
            os.makedirs(os.path.dirname(self.yaml_file), exist_ok=True)
            
            data = {'saved_locations': self.locations}
            with open(self.yaml_file, 'w') as file:
                yaml.dump(data, file, default_flow_style=False, sort_keys=True)
            return True
        except Exception as e:
            self.get_logger().error(f"Error saving locations file: {e}")
            return False
    
    def get_current_pose(self):
        """Get current robot pose in map frame"""
        try:
            # Get transform from map to base_link
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            
            # Create PoseStamped from transform
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            
            return pose
        except TransformException as e:
            self.get_logger().error(f"Error getting current pose: {e}")
            self.get_logger().error("Make sure your robot is publishing the map->base_link transform!")
            return None
    
    def save_location_callback(self, request, response):
        """Save current robot position as named location"""
        if not request.name.strip():
            response.success = False
            response.message = "Location name cannot be empty"
            return response
        
        # Clean the name
        clean_name = request.name.strip().lower().replace(' ', '_')
        
        # Get current pose
        current_pose = self.get_current_pose()
        if current_pose is None:
            response.success = False
            response.message = "Could not get current robot pose. Check TF transforms!"
            return response
        
        # Save to locations dict
        self.locations[clean_name] = {
            'position': {
                'x': current_pose.pose.position.x,
                'y': current_pose.pose.position.y,
                'z': current_pose.pose.position.z
            },
            'orientation': {
                'x': current_pose.pose.orientation.x,
                'y': current_pose.pose.orientation.y,
                'z': current_pose.pose.orientation.z,
                'w': current_pose.pose.orientation.w
            },
            'frame_id': current_pose.header.frame_id,
            'description': request.description if request.description.strip() else f"Saved location: {clean_name}"
        }
        
        # Save to file
        if self.save_locations_to_file():
            response.success = True
            response.message = f"Location '{clean_name}' saved successfully at ({current_pose.pose.position.x:.2f}, {current_pose.pose.position.y:.2f})"
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = f"Failed to save location '{clean_name}' to file"
        
        return response
    
    def get_location_callback(self, request, response):
        """Get pose for named location"""
        clean_name = request.name.strip().lower().replace(' ', '_')
        
        if clean_name not in self.locations:
            response.success = False
            response.message = f"Location '{clean_name}' not found"
            return response
        
        loc_data = self.locations[clean_name]
        
        try:
            # Create PoseStamped
            response.pose.header.frame_id = loc_data.get('frame_id', 'map')
            response.pose.header.stamp = self.get_clock().now().to_msg()
            response.pose.pose.position.x = float(loc_data['position']['x'])
            response.pose.pose.position.y = float(loc_data['position']['y'])
            response.pose.pose.position.z = float(loc_data['position']['z'])
            response.pose.pose.orientation.x = float(loc_data['orientation']['x'])
            response.pose.pose.orientation.y = float(loc_data['orientation']['y'])
            response.pose.pose.orientation.z = float(loc_data['orientation']['z'])
            response.pose.pose.orientation.w = float(loc_data['orientation']['w'])
            
            response.success = True
            response.message = f"Location '{clean_name}' found at ({response.pose.pose.position.x:.2f}, {response.pose.pose.position.y:.2f})"
        except (KeyError, ValueError, TypeError) as e:
            response.success = False
            response.message = f"Error parsing location data for '{clean_name}': {e}"
            self.get_logger().error(response.message)
        
        return response
    
    def list_locations_callback(self, request, response):
        """List all available locations"""
        response.names = list(self.locations.keys())
        response.descriptions = []
        
        for name in response.names:
            desc = self.locations[name].get('description', 'No description')
            response.descriptions.append(desc)
        
        response.success = True
        response.message = f"Found {len(response.names)} saved locations"
        
        return response
    
    def reload_locations_callback(self, request, response):
        """Reload locations from file"""
        self.locations = self.load_locations()
        self.get_logger().info(f"Reloaded {len(self.locations)} locations from file")
        return response

def main(args=None):
    rclpy.init(args=args)
    
    location_manager = LocationManager()
    
    # Use MultiThreadedExecutor for concurrent service calls
    executor = MultiThreadedExecutor()
    executor.add_node(location_manager)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        location_manager.get_logger().info("Location Manager shutting down...")
    finally:
        location_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()