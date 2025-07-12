// teleop_node.cpp
// Description:  
//   Read joystick or keyboard input and publish geometry_msgs/Twist on /cmd_vel.
//
// Outline:  
//   // includes (rclcpp, sensor_msgs/Joy or custom keyboard reader, geometry_msgs/Twist)  
//   class TeleopNode : public rclcpp::Node {  
//   public:  
//     TeleopNode();  
//   private:  
//     void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);  
//     rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;  
//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;  
//   };  
//   int main(int argc, char** argv) { … }
