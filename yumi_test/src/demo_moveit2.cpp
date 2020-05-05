#include <unistd.h>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include <moveit2_wrapper/moveit2_wrapper.hpp>
#include <rws_clients/robot_manager_client.hpp>
#include <rws_clients/grip_client.hpp>



// Global robot_manager
std::shared_ptr<rws_clients::RobotManagerClient> robot_manager;

// Home globally known
std::vector<double> home_l = {0.0, -2.26, 2.35, 0.52, 0.0, 0.52, 0.0};
std::vector<double> home_r = {0.0, -2.26, -2.35, 0.52, 0.0, 0.52, 0.0};

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

void spin(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exe)
{
  exe->spin();
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  // Create node to be used by Moveit2
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("onsket_nodenavn", "", node_options);
  moveit2_wrapper::Moveit2Wrapper moveit2(node);
  
  robot_manager = std::make_shared<rws_clients::RobotManagerClient>("robot_manager_client");
  if (!robot_manager->init())
  {
    std::cout << "Failed to initialize the robot manager client" << std::endl;
    return -1;
  }
  // Wait until robot is ready
  while (!robot_manager->robot_is_ready())
  {
    sleep(0.01);
  }

  // Initialize the left gripper
  auto grip_action_client_l = std::make_shared<rws_clients::GripClient>("grip_client_left", "/l");
  if (!grip_action_client_l->init())
  {
    std::cout << "Unable to initalize the left gripper client" << std::endl;
    return -1;
  }
  // Initialize the right gripper
  auto grip_action_client_r = std::make_shared<rws_clients::GripClient>("grip_client_right", "/r");
  if (!grip_action_client_r->init())
  {
    std::cout << "Unable to initalize the right gripper client" << std::endl;
    return -1;
  }


  if(!moveit2.init())
  {
    std::cout << "Initialization of moveit2 failed" << std::endl;
    return -1;
  }

  // Spin the moveit2 node countiously in another thread via an executor
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  auto future_handle = std::async(std::launch::async, spin, executor);


  // Start EGM mode
  if (!robot_manager->start_egm())
  {
    std::cout << "Failed to start egm" << std::endl;
    return -1;
  }

  // Launching the Moveit2 planning scene
  moveit2.launch_planning_scene();
  

  moveit2.pose_to_pose_motion("left_arm",  "gripper_l_base", {0.426,  0.137, 0.415, -160.89, 30.32, -113.20}, 2, true, false, false); // Euler Angles
  moveit2.pose_to_pose_motion("right_arm", "gripper_r_base", {0.464, -0.110, 0.469, -21.96, -19.29, -113.49}, 2, false, false, true); // Euler Angles

  // // Return home
  // moveit2.state_to_state_motion("left_arm", home_l, 2);
  // moveit2.state_to_state_motion("right_arm", home_r, 2);


  // moveit2.dual_arm_state_to_state_motion({-0.765, -0.929, 0.711, 0.819, 1.527, 0.025, -0.919}, 
  //                                        {0.798, -0.815, -0.503, 0.687, -1.197, 0.190, -2.138}, 2, true, true);


  std::cout << "Motion completed, please ctrl+c" << std::endl;
  while(1)
  {
    sleep(1);
  }
  return 0;
}
