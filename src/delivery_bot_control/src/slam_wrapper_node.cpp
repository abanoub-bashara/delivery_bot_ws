// slam_wrapper_node.cpp
// Description:
//   Wrap slam_toolbox (or custom SLAM) in C++: feed it /scan and /odom,
//   then publish nav_msgs/OccupancyGrid on /map and geometry_msgs/PoseWithCovarianceStamped on /slam_pose.
//
// Outline:
//   // includes (rclcpp, sensor_msgs/LaserScan, nav_msgs/OccupancyGrid, geometry_msgs/PoseWithCovarianceStamped)
//   class SlamWrapperNode : public rclcpp::Node {
//   public:
//     SlamWrapperNode();
//   private:
//     void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
//     void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
//     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
//     rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
//     rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
//     rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
//     // slam engine instance, update & publish methods
//   };
//   int main(int argc, char** argv) { … }
