#include <rws_clients/grip_client.hpp>


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::string node_name = "grip_client";
  auto node = rclcpp::Node::make_shared("grip_client_node");
  std::string ns = node->get_namespace();

  auto grip_action_client = std::make_shared<rws_clients::GripClient>(
    node_name, ns);

  grip_action_client->init();


  int percentage = std::stoi(argv[1]);
  grip_action_client->perform_grip(percentage); // 0=open, 100=closed

  while (!grip_action_client->is_goal_done()) 
  {
    rclcpp::spin_some(grip_action_client);
  }

  rclcpp::shutdown();
  return 0;
}