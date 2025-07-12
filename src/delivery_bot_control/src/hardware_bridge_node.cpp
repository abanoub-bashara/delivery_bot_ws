// hardware_bridge_node.cpp
// Description:
//   Bridge JointTrajectory commands into Gazebo (via ros2_control) and
//   republish resulting sensor_msgs/JointState for odometry.
//
// Outline:
//   // includes (rclcpp, trajectory_msgs/JointTrajectory, sensor_msgs/JointState)
//   class HardwareBridgeNode : public rclcpp::Node {
//   public:
//     HardwareBridgeNode();
//   private:
//     void traj_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
//     rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub_;
//     rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
//     // helper: send to gazebo, read back joint states
//   };
//   int main(int argc, char** argv) { … }
