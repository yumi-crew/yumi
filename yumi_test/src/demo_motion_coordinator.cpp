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
  if(!yumi_motion_coordinator->init()) return -1;

  // Spin the moveit2 node countiously in another thread via an executor
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(yumi_motion_coordinator->get_node());
  auto future_handle = std::async(std::launch::async, spin, executor);
  
  if(!yumi_motion_coordinator->activate())
  {
    std::cout << "ERROR: MotionCoordinator failed to activate." << std::endl;
    return -1;  
  }

  std::thread run_demo([yumi_motion_coordinator]() {});
  std::vector<double> pose =     {0.4,  0.1, -0.1, 30, 0, 0}; 
  std::vector<double> bin_pose = {0.4, -0.1, -0.2, 0, 0, 0 };
  int counter = 0; int retries = 3; double percentage = 1; double speed_scale = 1; double acc_scale = 1;
  
  //  ** TEST 1 ** 
  // while(!yumi_motion_coordinator->should_stop())
  // {
  //   yumi_motion_coordinator->add_object("screwdriver", pose, true);
  //   yumi_motion_coordinator->move_to_pose("left_arm", {0.4, 0.1, 0.3, 0, 0, 180}, true, 3, false, true, false);
  //   sleep(1);
  //   yumi_motion_coordinator->linear_move_to_pose("left_arm", {0.4, 0.1, 0.1, 0, 0, 180}, true, retries, false, true, true, percentage, speed_scale, acc_scale);
  //   sleep(1);
  //   yumi_motion_coordinator->linear_move_to_pose("left_arm", {0.4, 0.1, 0.3, 0, 0, 180}, true, 0, false, true, true, percentage, speed_scale, acc_scale);


  //   yumi_motion_coordinator->move_to_pose("left_arm", {0.4, -0.2, 0.3, 0, 0, 180}, true, 3, false, true, false);
  //   sleep(1);
  //   yumi_motion_coordinator->linear_move_to_pose("left_arm", {0.4, -0.2, 0.1, 0, 0, 180}, true, retries, false, true, true, percentage, speed_scale, acc_scale);
  //   sleep(1);
  //   yumi_motion_coordinator->linear_move_to_pose("left_arm", {0.4, -0.2, 0.3, 0, 0, 180}, true, 0, false, true, true, percentage, speed_scale, acc_scale);


  //   yumi_motion_coordinator->move_to_pose("left_arm", {0.4, 0, 0.3, 0, 0, 180}, true, 3, false, true, false);
  //   sleep(1);
  //   yumi_motion_coordinator->linear_move_to_pose("left_arm", {0.4, 0, 0.1, 0, 0, 180}, true, retries, false, true, true, percentage, speed_scale, acc_scale);
  //   sleep(1);
  //   yumi_motion_coordinator->linear_move_to_pose("left_arm", {0.4, 0, 0.3, 0, 0, 180}, true, 0, false, true, true, percentage, speed_scale, acc_scale);
    

  //   yumi_motion_coordinator->move_to_home("left_arm", 3);
  //   ++counter;
  //   std::cout << "++++++ " << counter << " rounds completed" <<std::endl;
  //   yumi_motion_coordinator->remove_object("screwdriver");
  // }


  // ** TEST 2 **
  while(!yumi_motion_coordinator->should_stop())
  // {
  //   yumi_motion_coordinator->add_object("screwdriver", pose, true);
  
  //   yumi_motion_coordinator->move_to_object("left_arm", "screwdriver", 0.2, 3, false, true, false);
  //   sleep(1);
  //   yumi_motion_coordinator->linear_move_to_object("left_arm", "screwdriver", 0.02, retries, false, true, true, percentage, speed_scale, acc_scale);
  //   sleep(1);
  //   yumi_motion_coordinator->linear_move_to_object("left_arm", "screwdriver", 0.2, 0, false, true, false, percentage, speed_scale, acc_scale);

  //   yumi_motion_coordinator->random_move_object("screwdriver", 0.2);
  //   yumi_motion_coordinator->move_to_object("left_arm", "screwdriver", 0.2, 3, false, true, false);
  //   sleep(1);
  //   yumi_motion_coordinator->linear_move_to_object("left_arm", "screwdriver", 0.02, retries, false, true, true, percentage, speed_scale, acc_scale);
  //   sleep(1);
  //   yumi_motion_coordinator->linear_move_to_object("left_arm", "screwdriver", 0.2, 0, false, true, true, percentage, speed_scale, acc_scale);
  //   sleep(1);
  
  //   yumi_motion_coordinator->move_to_home("left_arm", 3);
  //   ++counter;
  //   std::cout << "++++++ " << counter << " rounds completed" <<std::endl;
  //   yumi_motion_coordinator->remove_object("screwdriver");
  // }


  // ** TEST 3 **
  // while(!yumi_motion_coordinator->should_stop())
  // {
  //   yumi_motion_coordinator->add_object("screwdriver", pose, true);
  //   yumi_motion_coordinator->add_object("bin", bin_pose, true);
  //   //yumi_motion_coordinator->random_move_object("screwdriver", 0.1);
  
  //   yumi_motion_coordinator->move_to_object("left_arm", "screwdriver", 0.2, 3, false, true, false);
    
  //   sleep(1);
  //   if(!yumi_motion_coordinator->pick_object("left_arm", "screwdriver", 3, true, false, percentage)){ std::cout << "pick failed" << std::endl; }
  //   sleep(1);

  //   sleep(1);
  //   //if(!yumi_motion_coordinator->place_at_object("left_arm", "bin", 0, true, false, percentage)){ std::cout << "place failed" << std::endl; }
  //   sleep(1);

  //   yumi_motion_coordinator->move_to_home("left_arm", 3);
  //   ++counter;
  //   std::cout << "++++++ " << counter << " rounds completed" <<std::endl;
  //   yumi_motion_coordinator->remove_object("screwdriver");
  // }

  //std::cout << ">>>>> managed - " << counter << " - rounds with " << retries << " allowed retries and required percentage " << percentage << std::endl;
  std::cout << "Motion completed, please ctrl+c" << std::endl;
  while(1)
  {
    sleep(1);
  }
  return 0;
}
