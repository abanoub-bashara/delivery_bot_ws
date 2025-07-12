// map_saver_node.cpp
// Description:
//   Listen for a “save map” trigger (std_msgs/Empty on /save_map) and write the latest
//   nav_msgs/OccupancyGrid from /map out as PGM + YAML.
//
// Outline:
//   // includes (rclcpp, nav_msgs/OccupancyGrid, std_msgs/Empty, file I/O headers)
//   class MapSaverNode : public rclcpp::Node {
//   public:
//     MapSaverNode();
//   private:
//     void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
//     void save_callback(const std_msgs::msg::Empty::SharedPtr msg);
//     rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
//     rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr save_sub_;
//     nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
//     // helper: writePGM(), writeYAML()
//   };
//   int main(int argc, char** argv) { … }
