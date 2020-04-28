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


  std::thread run_demo([yumi_motion_coordinator]() {
    std::vector<double> pose = {0.4, 0.1, 0.0, 30, 0, 0}; std::vector<double> position;
    yumi_motion_coordinator->add_object("screwdriver", pose, true);
    sleep(2);
    position = yumi_motion_coordinator->random_move_object("screwdriver", {pose[0],pose[1],pose[2]}, 0.02);
    sleep(1);
    position = yumi_motion_coordinator->random_move_object("screwdriver", position, 0.02);
    sleep(1);
    position = yumi_motion_coordinator->random_move_object("screwdriver", position, 0.02);
    sleep(1);
    position = yumi_motion_coordinator->random_move_object("screwdriver", position, 0.02);
    sleep(1);
    position = yumi_motion_coordinator->random_move_object("screwdriver", position, 0.02);
  });

  sleep(2);
  yumi_motion_coordinator->move_to_object("left_arm", "screwdriver", 0.10, 3, false, true, true);
  yumi_motion_coordinator->grip_out("left");
  sleep(1);
  yumi_motion_coordinator->linear_move_to_object("left_arm", "screwdriver", 0.02, false, true);
  sleep(1);
  yumi_motion_coordinator->grip_in("left");
  sleep(1);
  yumi_motion_coordinator->linear_move_to_object("left_arm", "screwdriver", 0.10, false, true);
  yumi_motion_coordinator->move_to_home("left_arm", 3);

  std::cout << "Motion completed, please ctrl+c" << std::endl;
  while(1)
  {
    sleep(1);
  }
  return 0;
}
