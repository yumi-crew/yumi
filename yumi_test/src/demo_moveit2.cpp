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
  // // Turn off Motors
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
  rclcpp::NodeOptions node_options;
  
  // Initialize the robot manager client
  robot_manager = std::make_shared<rws_clients::RobotManagerClient>("robot_manager_client");
  std::cout << "before robot_manager->init()" << std::endl;
  if (!robot_manager->init())
  {
    std::cout << "Failed to initialize the robot manager client" << std::endl;
    return -1;
  }

  std::cout << "before robot_manager->robot_is_ready()" << std::endl;
  // Wait until robot is ready
  while (!robot_manager->robot_is_ready())
  {
    sleep(0.01);
  }

  
  // Initialize the left gripper
  auto grip_action_client_l = std::make_shared<rws_clients::GripClient>("grip_client_left", "/l");
  std::cout << "grip_action_client_l->init()" << std::endl;
  if (!grip_action_client_l->init())
  {
    std::cout << "Unable to initalize the left gripper client" << std::endl;
    return -1;
  }

  
  // Initialize the right gripper
  auto grip_action_client_r = std::make_shared<rws_clients::GripClient>("grip_client_right", "/r");
  std::cout << "before grip_action_client_r->init()" << std::endl;
  if (!grip_action_client_r->init())
  {
    std::cout << "Unable to initalize the right gripper client" << std::endl;
    return -1;
  }


  // Initialize moveit2
  std::cout << "before construction of moveit2" << std::endl;
  Moveit2Wrapper moveit2("onsket_nodenavn");
  
  // Spin the moveit2 node countiously in another thread via an executor
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(moveit2.get_node());
  auto future_handle = std::async(std::launch::async, spin, executor);


  std::cout << "before moveit2.init()" << std::endl;
  if(!moveit2.init())
  {
    std::cout << "Initialization of moveit2 failed" << std::endl;
    return -1;
  }

  std::cout << "before robot_manager->start_egm())" << std::endl;
  // Start EGM mode
  if (!robot_manager->start_egm())
  {
    std::cout << "Failed to start egm" << std::endl;
    return -1;
  }

  sleep(4); // wait to ensure joint_state_controller is publishing the joint states
  std::cout << "before moveit2.launch_planning_scene()" << std::endl;
  moveit2.launch_planning_scene();
  sleep(3); // wait to ensure the that moveit2 updates its representation of the robots position before the plannign scene is launched

  
  // std::cout << "RIGHT --- before moveit2.state_to_state_motion" << std::endl;
  // if(!moveit2.state_to_state_motion("right_arm", {0.798, -0.815, -0.503, 0.687, -1.197, 0.190, -2.138}, 2))
  // {
  //   std::cout << "RIGHT --- state_to_state_motion returned false" << std::endl;
  //   return -1;
  // } 
 


  // std::cout << "RIGHT --- moveit2.state_to_state_motion" << std::endl;
  // if(!moveit2.state_to_state_motion("right_arm", home_r, 2))
  // {
  //   std::cout << "RIGHT --- state_to_state_motion returned false" << std::endl;
  //   return -1;
  // } 


  // if(!moveit2.pose_to_pose_motion("right_arm", {0.464, -0.110, 0.469, -21.96, -19.29, -113.49}, 2))
  // {
  //   std::cout << "RIGHT --- pose_to_pose_motion returned false" << std::endl;
  //   return -1;
  // } 

  

  // std::cout << "LEFT --- before moveit2.state_to_state_motion" << std::endl;
  // if(!moveit2.state_to_state_motion("left_arm", {-0.765, -0.929, 0.711, 0.819, 1.527, 0.025, -0.919}, 2))
  // {
  //   std::cout << "LEFT --- state_to_state_motion returned false" << std::endl;
  //   return -1;
  // } 

  std::cout << "DUAL --- before moveit2.dual_arm_pose_to_pose_motion" << std::endl;
  if(!moveit2.dual_arm_pose_to_pose_motion({0.4269, 0.1373, 0.4158, -160.89, 30.32, -113.20}, {0.464, -0.110, 0.469, -21.96, -19.29, -113.49}, 10))
  {
    std::cout << "DUAL --- dual_arm_pose_to_pose_motion returned false" << std::endl;
    return -1;
  } 
  

  // std::cout << "LEFT --- moveit2.state_to_state_motion" << std::endl;
  // if(!moveit2.state_to_state_motion("left_arm", home_l, 2))
  // {
  //   std::cout << "LEFT --- state_to_state_motion returned false" << std::endl;
  //   return -1;
  // } 

  
  // moveit2.dual_arm_state_to_state_motion({-0.765, -0.929, 0.711, 0.819, 1.527, 0.025, -0.919}, 
  //                                        {0.798, -0.815, -0.503, 0.687, -1.197, 0.190, -2.138}, 10);
  // moveit2.dual_arm_state_to_state_motion(home_l, home_r, 10);



  std::cout << "motion completed, please ctrl+c" << std::endl;
  while(1)
  {
    sleep(1);
  }
  return 0;
}
