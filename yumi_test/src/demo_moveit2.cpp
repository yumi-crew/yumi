#include <unistd.h>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include <moveit2_demo_yumi/moveit2_demo_yumi.hpp>
#include <rws_clients/robot_manager_client.hpp>
#include <rws_clients/grip_client.hpp>

// Global joint names
std::array<std::string, 7> joint_names_l = {
    "yumi_joint_1_l", "yumi_joint_2_l", "yumi_joint_7_l", "yumi_joint_3_l",
    "yumi_joint_4_l", "yumi_joint_5_l", "yumi_joint_6_l"};
std::array<std::string, 7> joint_names_r = {
    "yumi_joint_1_r", "yumi_joint_2_r", "yumi_joint_7_r", "yumi_joint_3_r",
    "yumi_joint_4_r", "yumi_joint_5_r", "yumi_joint_6_r"};

// Global robot_manager
std::shared_ptr<rws_clients::RobotManagerClient> robot_manager;

// Ctr+C handler
void signal_callback_handler(int signum)
{
  std::cout << "Caught signal " << signum << std::endl;
  // Stop EGM
  robot_manager->stop_egm();
  // Turn off Motors
  robot_manager->stop_motors();
  // Terminate ros node
  rclcpp::shutdown();
  // Terminate program
  exit(signum);
}


int main(int argc, char** argv)
{
 
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;

  std::string ns_l = "/l";
  std::string ns_r = "/r";

  // Constructing the robot_manager
  robot_manager = std::make_shared<rws_clients::RobotManagerClient>("robot_manager_client");
  
  // Initialize the robot manager client
  if (!robot_manager->init())
  {
    printf("Failed to initialize the robot manager client\n");
    return -1;
  }
  // Wait until robot is ready
  while (!robot_manager->robot_is_ready())
  {
    sleep(0.01);
  }
  // Initialize the left gripper
  auto grip_action_client_l = std::make_shared<rws_clients::GripClient>("grip_client_left", ns_l);
  if (!grip_action_client_l->init())
  {
    printf("Unable to initalize the left gripper client\n");
    return -1;
  }
  // Initialize the right gripper
  auto grip_action_client_r = std::make_shared<rws_clients::GripClient>("grip_client_right", ns_r);
  if (!grip_action_client_r->init())
  {
    printf("Unable to initalize the right gripper client\n");
    return -1;
  }
  // Start EGM mode
  if (!robot_manager->start_egm())
  {
    printf("Failed to start egm\n");
    return -1;
  }



  // This enables loading undeclared parameters
  // best practice would be to declare parameters in the corresponding classes
  // and provide descriptions about expected use
  node_options.automatically_declare_parameters_from_overrides(true);
  

  
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("demo_moveit2", "", node_options);

  std::cout << "... before construction of MoveItCppDemo" << std::endl;
  MoveItCppDemo demo(node);
  std::cout << "... after construction of MoveItCppDemo" << std::endl;

  sleep(10);

  std::cout << "... before thread run_demo" << std::endl;
  // spin out a thread running the MoveItCppDemo::run()
  std::thread run_demo([&demo]() {
    // Let RViz initialize before running demo
    rclcpp::sleep_for(std::chrono::seconds(10));
    std::cout << "... before demo.run()" << std::endl;
    demo.run();
  });

  rclcpp::spin(node);
  run_demo.join();
  

  printf("motion completed, please ctrl+c\n");
  while (1)
  {
    sleep(1);
  }
  return 0;
}
