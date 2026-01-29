#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import re
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from delivery_bot_nav.srv import GetLocation, SaveLocation, ListLocations
from std_srvs.srv import Empty
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import threading
import time

class CommandInterface(Node):
    def __init__(self):
        super().__init__('command_interface')
        
        # Callback group for concurrent operations
        self.callback_group = ReentrantCallbackGroup()
        
        # Wait for location manager services
        self.get_logger().info("Waiting for location manager services...")
        
        # Service clients
        self.get_location_client = self.create_client(
            GetLocation, 'location_manager/get_location', 
            callback_group=self.callback_group)
        self.save_location_client = self.create_client(
            SaveLocation, 'location_manager/save_location',
            callback_group=self.callback_group)
        self.list_locations_client = self.create_client(
            ListLocations, 'location_manager/list_locations',
            callback_group=self.callback_group)
        self.reload_locations_client = self.create_client(
            Empty, 'location_manager/reload_locations',
            callback_group=self.callback_group)
        
        # Wait for services
        self.wait_for_services()
        
        # Nav2 action client
        self.nav_action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.callback_group)
        
        self.get_logger().info("Waiting for Nav2 action server...")
        if self.nav_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().info("Nav2 action server found!")
        else:
            self.get_logger().warning("Nav2 action server not found. Navigation commands will fail.")
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("🤖 ROBOT COMMAND INTERFACE READY! 🤖")
        self.get_logger().info("=" * 50)
        self.get_logger().info("Available commands:")
        self.get_logger().info("  📍 go to <location>      - Navigate to saved location")
        self.get_logger().info("  💾 save location <n>  - Save current position as location")
        self.get_logger().info("  📋 list locations        - Show all saved locations")
        self.get_logger().info("  🛑 stop                  - Cancel current navigation")
        self.get_logger().info("  ❓ help                  - Show this help message")
        self.get_logger().info("  👋 quit/exit             - Exit command interface")
        self.get_logger().info("=" * 50)
        
        # Show available locations
        self.show_available_locations()
        
        # Current navigation goal handle
        self.current_goal_handle = None
        self.navigation_active = False
    
    def wait_for_services(self):
        """Wait for all required services to be available"""
        services = [
            (self.get_location_client, 'location_manager/get_location'),
            (self.save_location_client, 'location_manager/save_location'),
            (self.list_locations_client, 'location_manager/list_locations'),
            (self.reload_locations_client, 'location_manager/reload_locations')
        ]
        
        for client, name in services:
            self.get_logger().info(f'Waiting for {name} service...')
            while not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().info(f'Still waiting for {name} service...')
            self.get_logger().info(f'✅ {name} service available!')
    
    def show_available_locations(self):
        """Display available saved locations"""
        try:
            request = ListLocations.Request()
            future = self.list_locations_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.done():
                response = future.result()
                if response.success and response.names:
                    self.get_logger().info("\n📍 Available locations:")
                    for i, (name, desc) in enumerate(zip(response.names, response.descriptions)):
                        self.get_logger().info(f"  {i+1}. {name} - {desc}")
                else:
                    self.get_logger().info("📍 No saved locations found. Use 'save location <n>' to create some!")
            else:
                self.get_logger().warning("Timeout getting locations list")
        except Exception as e:
            self.get_logger().error(f"Error getting locations list: {e}")
    
    def parse_command(self, command_text):
        """Parse text command and execute appropriate action"""
        command = command_text.strip().lower()
        
        if not command:
            return True
        
        # Help command
        if command in ['help', 'h', '?']:
            self.show_help()
            return True
        
        # Quit commands
        if command in ['quit', 'exit', 'q']:
            self.get_logger().info("👋 Goodbye!")
            return False
        
        # Stop navigation
        if command in ['stop', 'cancel', 'halt']:
            self.stop_navigation()
            return True
        
        # List locations
        if command in ['list', 'list locations', 'locations', 'show locations']:
            self.show_available_locations()
            return True
        
        # Navigation commands: "go to <location>", "navigate to <location>", "move to <location>"
        nav_patterns = [
            r'^(?:go to|navigate to|move to|goto)\s+(.+)$',
            r'^go\s+(.+)$'
        ]
        
        for pattern in nav_patterns:
            match = re.match(pattern, command)
            if match:
                location_name = match.group(1).strip()
                self.navigate_to_location(location_name)
                return True
        
        # Save location: "save location <n>", "save <n>"
        save_patterns = [
            r'^save location\s+(.+)$',
            r'^save\s+(.+)$'
        ]
        
        for pattern in save_patterns:
            match = re.match(pattern, command)
            if match:
                location_name = match.group(1).strip()
                self.save_current_location(location_name)
                return True
        
        # If no pattern matched
        self.get_logger().warning(f"❌ Unknown command: '{command_text}'")
        self.get_logger().info("Type 'help' for available commands")
        return True
    
    def navigate_to_location(self, location_name):
        """Navigate to specified location using Nav2"""
        if self.navigation_active:
            self.get_logger().warning("🚫 Navigation already in progress! Use 'stop' to cancel first.")
            return
            
        try:
            # Get location pose
            request = GetLocation.Request()
            request.name = location_name
            
            future = self.get_location_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if not future.done():
                self.get_logger().error("⏰ Timeout getting location")
                return
                
            response = future.result()
            
            if not response.success:
                self.get_logger().warning(f"❌ {response.message}")
                self.suggest_similar_locations(location_name)
                return
            
            # Create Nav2 goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = response.pose
            
            self.get_logger().info(f"🚀 Navigating to '{location_name}'...")
            self.navigation_active = True
            
            # Send goal to Nav2
            future = self.nav_action_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if not future.done():
                self.get_logger().error("⏰ Timeout sending navigation goal")
                self.navigation_active = False
                return
            
            self.current_goal_handle = future.result()
            
            if not self.current_goal_handle.accepted:
                self.get_logger().error(f"❌ Navigation goal to '{location_name}' was rejected")
                self.navigation_active = False
                return
            
            self.get_logger().info(f"✅ Navigation goal to '{location_name}' accepted")
            
            # Wait for result in a separate thread to not block command input
            result_thread = threading.Thread(
                target=self._wait_for_navigation_result, 
                args=(location_name,), 
                daemon=True
            )
            result_thread.start()
                
        except Exception as e:
            self.get_logger().error(f"💥 Error navigating to location: {e}")
            self.navigation_active = False
    
    def _wait_for_navigation_result(self, location_name):
        """Wait for navigation result in separate thread"""
        try:
            result_future = self.current_goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)
            
            self.navigation_active = False
            
            if result_future.done():
                result = result_future.result()
                if result.status == 4:  # SUCCEEDED
                    self.get_logger().info(f"🎯 Successfully reached '{location_name}'!")
                elif result.status == 5:  # CANCELED
                    self.get_logger().info(f"🛑 Navigation to '{location_name}' was cancelled")
                else:
                    self.get_logger().warning(f"⚠️  Navigation to '{location_name}' failed (status: {result.status})")
            else:
                self.get_logger().warning(f"⏰ Navigation to '{location_name}' timed out")
                
        except Exception as e:
            self.get_logger().error(f"💥 Error waiting for navigation result: {e}")
            self.navigation_active = False
    
    def save_current_location(self, location_name):
        """Save current robot position as named location"""
        try:
            request = SaveLocation.Request()
            request.name = location_name
            request.description = f"Saved via command interface"
            
            future = self.save_location_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.done():
                response = future.result()
                if response.success:
                    self.get_logger().info(f"💾 {response.message}")
                else:
                    self.get_logger().warning(f"❌ {response.message}")
            else:
                self.get_logger().error("⏰ Timeout saving location")
                
        except Exception as e:
            self.get_logger().error(f"💥 Error saving location: {e}")
    
    def stop_navigation(self):
        """Cancel current navigation goal"""
        if self.current_goal_handle and self.navigation_active:
            try:
                future = self.current_goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                self.get_logger().info("🛑 Navigation cancelled")
                self.navigation_active = False
            except Exception as e:
                self.get_logger().error(f"💥 Error cancelling navigation: {e}")
        else:
            self.get_logger().info("ℹ️  No active navigation to cancel")
    
    def suggest_similar_locations(self, requested_name):
        """Suggest similar location names if exact match not found"""
        try:
            request = ListLocations.Request()
            future = self.list_locations_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if not future.done():
                return
                
            response = future.result()
            if response.success and response.names:
                # Simple substring matching for suggestions
                suggestions = [name for name in response.names 
                             if requested_name.lower() in name.lower() or name.lower() in requested_name.lower()]
                
                if suggestions:
                    self.get_logger().info(f"💡 Did you mean one of these? {', '.join(suggestions)}")
                else:
                    self.get_logger().info("📍 Available locations:")
                    for name in response.names:
                        self.get_logger().info(f"  - {name}")
        except Exception as e:
            self.get_logger().error(f"Error getting location suggestions: {e}")
    
    def show_help(self):
        """Show available commands"""
        self.get_logger().info("\n" + "=" * 40)
        self.get_logger().info("🤖 COMMAND INTERFACE HELP")
        self.get_logger().info("=" * 40)
        self.get_logger().info("📍 Navigation commands:")
        self.get_logger().info("  go to <location>      - Navigate to saved location")
        self.get_logger().info("  navigate to <location> - Navigate to saved location")  
        self.get_logger().info("  move to <location>    - Navigate to saved location")
        self.get_logger().info("  go <location>         - Navigate to saved location")
        self.get_logger().info("")
        self.get_logger().info("💾 Location management:")
        self.get_logger().info("  save location <n>  - Save current position")
        self.get_logger().info("  save <n>           - Save current position")
        self.get_logger().info("  list locations        - Show all saved locations")
        self.get_logger().info("")
        self.get_logger().info("🎮 Control commands:")
        self.get_logger().info("  stop                  - Cancel current navigation")
        self.get_logger().info("  help                  - Show this help")
        self.get_logger().info("  quit/exit             - Exit command interface")
        self.get_logger().info("=" * 40)

def main(args=None):
    rclpy.init(args=args)
    
    cmd_interface = CommandInterface()
    
    # Use MultiThreadedExecutor for handling action clients
    executor = MultiThreadedExecutor()
    executor.add_node(cmd_interface)
    
    # Start executor in a separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    # Command loop
    try:
        cmd_interface.get_logger().info("\nType commands below (type 'help' for options):")
        while rclpy.ok():
            try:
                user_input = input("\n🤖 Robot> ").strip()
                if user_input:
                    if not cmd_interface.parse_command(user_input):
                        break  # User wants to quit
            except KeyboardInterrupt:
                cmd_interface.get_logger().info("\n👋 Goodbye!")
                break
            except EOFError:
                cmd_interface.get_logger().info("\n👋 Goodbye!")
                break
            except Exception as e:
                cmd_interface.get_logger().error(f"💥 Error processing command: {e}")
    finally:
        cmd_interface.get_logger().info("Shutting down command interface...")
        cmd_interface.destroy_node()
        rclpy.shutdown()
        executor_thread.join(timeout=2.0)

if __name__ == '__main__':
    main()