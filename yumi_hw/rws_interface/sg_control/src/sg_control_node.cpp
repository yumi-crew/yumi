#include <sg_control/sg_control.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  auto sg_gripper = std::make_shared<sg_control::SgControl>(options, "192.168.125.1");
  sg_gripper->init();

  std::thread monitor_gripper([sg_gripper]()
  {
    while(rclcpp::ok()){ sg_gripper->publish_gripper_position(); }
  });

  rclcpp::spin(sg_gripper);
    
  rclcpp::shutdown();
  return 0;
}