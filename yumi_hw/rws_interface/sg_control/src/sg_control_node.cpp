#include <sg_control/sg_control.hpp>



int main(int argc, char* argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  auto grip_action_server = std::make_shared<sg_control::SgControl>(options, "192.168.125.1");

  grip_action_server->init();
  rclcpp::spin(grip_action_server);
  
  
  
  
  rclcpp::shutdown();
  printf("done\n");
  return 0;
}