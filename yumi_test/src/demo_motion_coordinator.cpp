#include <motion_coordinator/motion_coordinator.hpp>

std::shared_ptr<motion_coordinator::MotionCoordinator> yumi_motion_coordinator;

// Ctr+C handler
void signal_callback_handler(int signum)
{
  std::cout << "Caught signal " << signum << std::endl;
  // Terminate EGM session
  yumi_motion_coordinator->terminate_egm_session();
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

  yumi_motion_coordinator = std::make_shared<motion_coordinator::MotionCoordinator>("onsket_nodenavn");

  if(!yumi_motion_coordinator->init())
  {
    std::cout << "ERROR: MotionCoordinator failed to initialize." << std::endl;
    return -1;
  }

  // Spin the moveit2 node countiously in another thread via an executor
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(yumi_motion_coordinator->get_node());
  auto future_handle = std::async(std::launch::async, spin, executor);
  
  if(!yumi_motion_coordinator->activate())
  {
    std::cout << "ERROR: MotionCoordinator failed to activate." << std::endl;
    return -1;  
  }


  // std::thread run_demo([yumi_motion_coordinator]() {
  //   std::vector<double> bin_pos;
  //   while(!yumi_motion_coordinator->planning_component_in_motion("left_arm")){ sleep(0.1); }
  //   sleep(1);
  //   bin_pos = yumi_motion_coordinator->random_move_bin({0.40, 0.0, -0.11});
  //   sleep(0.5);
  //   while(!yumi_motion_coordinator->planning_component_in_motion("left_arm")){ sleep(0.1); }
  //   sleep(1);
  //   bin_pos = yumi_motion_coordinator->random_move_bin(bin_pos);
  // });


  yumi_motion_coordinator->move_to_pose("right_arm",  {0.426,  0, 0.215, 0, 0, 180}, true, 2, true, true, false);
  yumi_motion_coordinator->linear_move_to_pose("right_arm", {0.426,  0, 0.115, 0, 0, 180}, true, true, true); //down
  yumi_motion_coordinator->linear_move_to_pose("right_arm", {0.426,  0, 0.215, 0, 0, 180}, true, true, true); //up
  
  yumi_motion_coordinator->move_to_home("right_arm", 2, true, true, false);

  //yumi_motion_coordinator->find_object("bin");


  std::cout << "Motion completed, please ctrl+c" << std::endl;
  while(1)
  {
    sleep(1);
  }
  return 0;
}
