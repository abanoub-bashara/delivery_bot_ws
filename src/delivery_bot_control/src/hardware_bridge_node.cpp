// sudo apt install pigpio-tools libpigpio-dev

#include <rclcpp/rclcpp.hpp>
// … any other includes …



#include <rclcpp/rclcpp.hpp>

class HardwareBridge : public rclcpp::Node {
public:
  HardwareBridge()
  : Node("hardware_bridge")
  {
    // your constructor will go here later
  }
};


int main(int argc, char ** argv)
{
  // 1) Initialize ROS 2
  rclcpp::init(argc, argv);

  // 2) Create your node (you’ll implement this class above)
  auto node = std::make_shared<HardwareBridge>();

  // 3) Spin until shutdown
  rclcpp::spin(node);

  // 4) Clean up and exit
  rclcpp::shutdown();
  return 0;
}
