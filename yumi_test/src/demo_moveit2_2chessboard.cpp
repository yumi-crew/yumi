#include <unistd.h>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include <moveit2_wrapper/moveit2_wrapper.hpp>
#include <rws_clients/robot_manager_client.hpp>
#include <rws_clients/grip_client.hpp>

#include "pose_estimation_manager.hpp"

using namespace std::chrono_literals;

// Global robot_manager
std::shared_ptr<rws_clients::RobotManagerClient> robot_manager;

// Home globally known
std::vector<double> home_l = {0.0, -2.26, 2.35, 0.52, 0.0, 0.52, 0.0};
std::vector<double> home_r = {0.0, -2.26, -2.35, 0.52, 0.0, 0.52, 0.0};

std::shared_ptr<PoseEstimationManager> pose_estimation_manager; // declaration for signal handling (ctrl+c)


// Ctr+C handler
void signal_callback_handler(int signum)
{
  std::cout << "Caught signal " << signum << std::endl;
  // shutdown lifecycle nodes
  pose_estimation_manager->change_state(
      "zivid_camera", lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, 10s);
  pose_estimation_manager->change_state(
      "pose_estimation", lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, 10s);
  pose_estimation_manager->change_state(
      "zivid_camera", lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP, 10s);
  pose_estimation_manager->change_state(
      "pose_estimation", lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP, 10s);
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

int main(int argc, char **argv)
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
  if (!moveit2.init())
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

  // zivid + pose_estimation
  pose_estimation_manager = std::make_shared<PoseEstimationManager>("pose_estimation_manager");

  rclcpp::executors::MultiThreadedExecutor exe;
  exe.add_node(pose_estimation_manager);

  auto state1 = pose_estimation_manager->get_state("zivid_camera", 3s);
  auto state2 = pose_estimation_manager->get_state("pose_estimation", 3s);

  auto transition_success1 = pose_estimation_manager->change_state(
      "zivid_camera", lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, 5s);
  auto transition_success2 = pose_estimation_manager->change_state(
      "pose_estimation", lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, 5s);

  pose_estimation_manager->add_camera_parameter("zivid_camera.capture.frame_0.iris", rclcpp::ParameterValue(17));
  pose_estimation_manager->add_camera_parameter("zivid_camera.capture.frame_1.iris", rclcpp::ParameterValue(25));
  pose_estimation_manager->add_camera_parameter("zivid_camera.capture.frame_2.iris", rclcpp::ParameterValue(30));
  pose_estimation_manager->call_set_param_srv(10s);

  auto state3 = pose_estimation_manager->get_state("zivid_camera", 3s);
  auto state4 = pose_estimation_manager->get_state("pose_estimation", 3s);

  auto transition_success3 = pose_estimation_manager->change_state(
      "zivid_camera", lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, 10s);
  auto transition_success4 = pose_estimation_manager->change_state(
      "pose_estimation", lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, 5s);
  auto state5 = pose_estimation_manager->get_state("zivid_camera", 3s);
  auto state6 = pose_estimation_manager->get_state("pose_estimation", 3s);

  sleep(4); // wait to ensure joint_state_controller is publishing the joint states
  std::cout << "before moveit2.launch_planning_scene()" << std::endl;
  moveit2.launch_planning_scene();
  sleep(3); // wait to ensure the that moveit2 updates its representation of the robots position before the plannign scene is launched

  // Movements
  //--------------------------------------------------------------------------------------------------------------------

  // Go to home position (start pose)
  std::cout << "RIGHT --- moveit2.state_to_state_motion" << std::endl;
  if (!moveit2.state_to_state_motion("right_arm", home_r, 2, false))
  {
    std::cout << "RIGHT --- state_to_state_motion returned false" << std::endl;
    return -1;
  }
  std::cout << "LEFT --- moveit2.state_to_state_motion" << std::endl;
  if (!moveit2.state_to_state_motion("left_arm", home_l, 2, false))
  {
    std::cout << "LEFT --- state_to_state_motion returned false" << std::endl;
    return -1;
  }
  bool cap_success{false};
  bool est_success{false};
  std::vector<float> grasp_pose; grasp_pose.resize(7);
  while (1)
  {
    std::cout << "before call_capture_srv" << std::endl;
    cap_success = pose_estimation_manager->call_capture_srv(5s);
    std::cout << "before call_estimate_pose_srv" << std::endl;
    est_success = pose_estimation_manager->call_estimate_pose_srv(10s);
    if (est_success)
    {
      std::cout << "before get_graspable_chessboard_pose" << std::endl;
      grasp_pose = pose_estimation_manager->pose_transformer->chessboard_pose_to_base_frame(0.05, false);

      std::vector<double> pose; pose.resize(7);
      std::cout << "before copy_n" << std::endl;
      std::copy_n(grasp_pose.begin(), 7, pose.begin());
      // blocking_cart_p2p_motion_right(pose);
      std::cout << "before pose_to_pose_motion" << std::endl;
      if(!moveit2.pose_to_pose_motion("right_arm", pose, 2, true, true))
      {
        std::cout << "pose_to_pose motion failed" << std::endl;
      }
      sleep(2);
      if (!moveit2.state_to_state_motion("right_arm", home_r, 2, true))
      {
        std::cout << "RIGHT --- state_to_state_motion returned false" << std::endl;
      }

    }
  }
  return 0;
}
