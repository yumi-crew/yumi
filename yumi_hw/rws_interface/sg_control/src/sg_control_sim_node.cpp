#include <sg_control/sg_control_sim.h>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("sg_control");

  auto grip_action_server = std::make_shared<sg_control::SgControl>(node);

  grip_action_server->init();
  rclcpp::spin(grip_action_server->get_node());
  
  rclcpp::shutdown();
  printf("done\n");
  return 0;
}