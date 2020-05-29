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

int main(int argc, char **argv)
{
  // Ctrl+C handler
  signal(SIGINT, signal_callback_handler);

  rclcpp::init(argc, argv);

  // Initialize yumi motion coordinator
  yumi_motion_coordinator = std::make_shared<motion_coordinator::MotionCoordinator>("motion_coordinator");
  if (!yumi_motion_coordinator->init())
    return -1;

  // Spin the moveit2 node countiously in another thread via an executor
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(yumi_motion_coordinator->get_node());
  auto future_handle = std::async(std::launch::async, spin, executor);

  // Activate yumi, EGM, Moveit2
  if (!yumi_motion_coordinator->activate())
  {
    std::cout << "ERROR: MotionCoordinator failed to activate." << std::endl;
    return -1;
  }

  
  while (1)
  {
    yumi_motion_coordinator->move_to_pose("right_arm", {0.3, -0.2, 0.3, 0, 0, 180}, true, 5);
    sleep(40);
    yumi_motion_coordinator->move_to_pose("right_arm", {0.5, -0.2, 0.3, 0, 0, 180}, true, 5);
    sleep(40);
  }

  std::cout << "Motion completed, please ctrl+c" << std::endl;
  while (1)
  {
    sleep(1);
  }
  return 0;
}
