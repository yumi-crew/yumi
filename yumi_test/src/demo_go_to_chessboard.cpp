#include <unistd.h>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <angles/angles.h>
#include <sensor_msgs/msg/joint_state.hpp>

#include <ros2_control_interfaces/msg/joint_control.hpp>
#include <rws_clients/grip_client.hpp>
#include <rws_clients/robot_manager_client.hpp>
#include <kdl_wrapper/kdl_wrapper.h>

#include "object_pose_estimator.hpp"
#include "pose_listener.hpp"

using namespace std::chrono_literals;

// Global joint names
std::array<std::string, 7> joint_names_l = {
    "yumi_joint_1_l", "yumi_joint_2_l", "yumi_joint_7_l", "yumi_joint_3_l",
    "yumi_joint_4_l", "yumi_joint_5_l", "yumi_joint_6_l"};
std::array<std::string, 7> joint_names_r = {
    "yumi_joint_1_r", "yumi_joint_2_r", "yumi_joint_7_r", "yumi_joint_3_r",
    "yumi_joint_4_r", "yumi_joint_5_r", "yumi_joint_6_r"};
// Global storage of the latest joint states
std::array<double, 7> recieved_joint_state_l, recieved_joint_state_r;
KDL::JntArray recieved_joint_state_array_l, recieved_joint_state_array_r;
// Global robot_manager
std::shared_ptr<rws_clients::RobotManagerClient> robot_manager;
// Global kdl_wrapper
std::shared_ptr<KdlWrapper> kdl_wrapper;
// Global nodes
std::shared_ptr<rclcpp::Node> node_s_l;
std::shared_ptr<rclcpp::Node> node_s_r;
std::shared_ptr<rclcpp::Node> node_p_l;
std::shared_ptr<rclcpp::Node> node_p_r;
// Gloal publishers and subscriptions
std::shared_ptr<rclcpp::Publisher<ros2_control_interfaces::msg::JointControl>> joint_command_publisher_r;
std::shared_ptr<rclcpp::Publisher<ros2_control_interfaces::msg::JointControl>> joint_command_publisher_l;
std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::JointState>> joint_state_subscription_r;
std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::JointState>> joint_state_subscription_l;

std::shared_ptr<ObjectPoseEstimator> pose_estimation_manager; // declaration for signal handling (ctrl+c)
std::shared_ptr<PoseListener> pose_listener;

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
  // Turn off Motors
  robot_manager->stop_motors();
  // Terminate ros node
  rclcpp::shutdown();
  // Terminate program
  exit(signum);
}

void callback_l(sensor_msgs::msg::JointState::UniquePtr msg)
{
  for (int i = 0; i < 7; ++i)
  {
    recieved_joint_state_l[i] = msg->position[i];
    recieved_joint_state_array_l(i) = msg->position[i];
  }
}
void callback_r(sensor_msgs::msg::JointState::UniquePtr msg)
{
  for (int i = 0; i < 7; ++i)
  {
    recieved_joint_state_r[i] = msg->position[i];
    recieved_joint_state_array_r(i) = msg->position[i];
  }
}

// For ZYX euler angles
KDL::Frame vec_and_euler_angles_to_frame(std::array<double, 6> pose)
{
  KDL::Vector vec(pose[0], pose[1], pose[2]);
  KDL::Rotation rot = KDL::Rotation().EulerZYX(pose[3], pose[4], pose[5]);
  return KDL::Frame(rot, vec);
}

KDL::Frame vec_and_quat_to_frame(std::vector<float> pose)
{
  KDL::Vector vec(pose[0], pose[1], pose[2]);
  KDL::Rotation rot = KDL::Rotation().Quaternion((double)pose[3], (double)pose[4], (double)pose[5], (double)pose[6]);
  return KDL::Frame(rot, vec);
}

KDL::JntArray generate_q_seed(std::string arm)
{
  KDL::JntArray q_seed(7);

  if (arm == "r")
  {
    for (int i = 0; i < 7; ++i)
    {
      q_seed(i) = recieved_joint_state_r[i];
    }
    return q_seed;
  }
  else if (arm == "l")
  {
    for (int i = 0; i < 7; ++i)
    {
      q_seed(i) = recieved_joint_state_l[i];
    }
    return q_seed;
  }
  else
  {
    std::cout << "Error while generating q_seed, invalid arm choice" << std::endl;
    return q_seed;
  }
}

void generate_msg_and_publish_r(KDL::JntArray &q_config)
{
  ros2_control_interfaces::msg::JointControl cmd;
  for (int i = 0; i < 7; ++i)
  {
    cmd.joints.push_back(joint_names_r[i]);
    cmd.goals.push_back(q_config(i));
  }
  joint_command_publisher_r->publish(cmd);
  rclcpp::spin_some(node_p_r);
}

void generate_msg_and_publish_l(KDL::JntArray &q_config)
{
  ros2_control_interfaces::msg::JointControl cmd;
  for (int i = 0; i < 7; ++i)
  {
    cmd.joints.push_back(joint_names_l[i]);
    cmd.goals.push_back(q_config(i));
  }
  joint_command_publisher_l->publish(cmd);
  rclcpp::spin_some(node_p_l);
}

// Not used
double sum_error(std::array<double, 7> &actual, KDL::JntArray &goal)
{
  double total_error = 0;
  for (int i = 0; i < 7; ++i)
  {
    total_error += std::abs(actual[i] - goal(i));
  }
  return total_error;
}

void busy_wait_until_reached(KDL::JntArray &goal, std::string arm, double allowed_joint_error)
{
  if (arm == "r")
  {
    while (!Equal(recieved_joint_state_array_r, goal, allowed_joint_error))
    {
      sleep(0.004); //250hz
    }
    for (int i = 0; i < 7; ++i)
    {
      //std::cout << "Joint " << i << ", Actual: " << angles::to_degrees(recieved_joint_state_r[i])
      // << " Goal: " << angles::to_degrees(goal(i)) << " Error: " << std::abs(angles::to_degrees(recieved_joint_state_r[i]) - angles::to_degrees(goal(i))) << std::endl;
    }
  }
  else if (arm == "l")
  {
    while (!Equal(recieved_joint_state_array_l, goal, allowed_joint_error))
    {
      sleep(0.004); //250hz
    }
  }
  else
  {
    std::cout << "Error while checking if goal is reached, invalid arm choice" << std::endl;
  }
}

// Blocking cartesian point-to-point motion. Converts desired end-effector pose to
// a joint configuration resulting in the pose.
void blocking_cart_p2p_motion_right(std::array<double, 6> pose)
{
  KDL::Frame pose_frame = vec_and_euler_angles_to_frame(pose);
  KDL::JntArray q_seed = generate_q_seed("r");
  KDL::JntArray q_config;
  bool success{false};
  // while (!success)
  // {
  //   try
  //   {
      q_config = kdl_wrapper->inverse_kinematics_right(pose_frame, q_seed);
    //   success = true;
    // }
    // catch (const std::exception &e)
    // {
    //   std::cout << e.what() << std::endl;
    //   return;
    // }
  // }
  generate_msg_and_publish_r(q_config);
  busy_wait_until_reached(q_config, "r", angles::from_degrees(0.01));
}

// Blocking cartesian point-to-point motion. Converts desired end-effector pose to
// a joint configuration resulting in the pose.
void blocking_cart_p2p_motion_left(std::array<double, 6> pose)
{
  KDL::Frame pose_frame = vec_and_euler_angles_to_frame(pose);
  KDL::JntArray q_seed = generate_q_seed("l");
  KDL::JntArray q_config;
  try
  {
    q_config = kdl_wrapper->inverse_kinematics_left(pose_frame, q_seed);
  }
  catch (const std::exception &e)
  {
    std::cout << e.what() << std::endl;
    return;
  }
  generate_msg_and_publish_l(q_config);
  busy_wait_until_reached(q_config, "l", angles::from_degrees(0.01));
}

KDL::JntArray array_to_joint_array(std::array<double, 7> &array)
{
  KDL::JntArray jnt_array(7);
  for (int i = 0; i < 7; ++i)
  {
    jnt_array(i) = angles::from_degrees(array[i]);
  }
  return jnt_array;
}

void go_to_home_pos()
{
  std::array<double, 7> goal_l = {0.0, -130.0, 135.0, 30.0, 0.0, 30.0, 0.0};
  std::array<double, 7> goal_r = {0.0, -130.0, -135.0, 30.0, 0.0, 30.0, 0.0};
  KDL::JntArray q_l = array_to_joint_array(goal_l);
  KDL::JntArray q_r = array_to_joint_array(goal_r);
  generate_msg_and_publish_l(q_l);
  generate_msg_and_publish_r(q_r);
  busy_wait_until_reached(q_r, "r", angles::from_degrees(0.01));
  busy_wait_until_reached(q_l, "l", angles::from_degrees(0.01));
}

void spin(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exe)
{
  exe->spin();
}

// Joints are always given as {1,2,7,3,4,5,6}
int main(int argc, char *argv[])
{
  // Ctrl+C handler
  signal(SIGINT, signal_callback_handler);

  // Setup
  rclcpp::init(argc, argv);
  node_s_l = rclcpp::Node::make_shared("sub_left");
  node_s_r = rclcpp::Node::make_shared("sub_right");
  node_p_l = rclcpp::Node::make_shared("pub_left");
  node_p_r = rclcpp::Node::make_shared("pub_right");
  std::string ns_l = "/l";
  std::string ns_r = "/r";
  recieved_joint_state_array_l.resize(7);
  recieved_joint_state_array_r.resize(7);

  // Constructing the robot_manager
  robot_manager = std::make_shared<rws_clients::RobotManagerClient>("robot_manager_client");

  // Finding path to urdf
  char buf[20];
  if (getlogin_r(buf, 20) != 0)
  {
    printf("Unable to find username\n");
    return -1;
  }
  std::string username = buf;
  std::string path = "/home/" + username + "/abb_ws/src/yumi/yumi_description/urdf/yumi.urdf";

  // Loading URDf and constructing the KdlWrapper
  urdf::Model robot_model;
  if (!robot_model.initFile(path))
  {
    printf("unable to load urdf\n");
    return -1;
  }
  kdl_wrapper = std::make_shared<KdlWrapper>(robot_model);
  if (!kdl_wrapper->init())
  {
    printf("initialization of kdlWrapper failed\n");
    return -1;
  }

  joint_state_subscription_l = node_s_l->create_subscription<sensor_msgs::msg::JointState>(
      ns_l + "/joint_states",
      10,
      callback_l);
  joint_command_publisher_l = node_p_l->create_publisher<ros2_control_interfaces::msg::JointControl>(
      ns_l + "/joint_commands",
      10);
  joint_state_subscription_r = node_s_r->create_subscription<sensor_msgs::msg::JointState>(
      ns_r + "/joint_states",
      10,
      callback_r);
  joint_command_publisher_r = node_p_r->create_publisher<ros2_control_interfaces::msg::JointControl>(
      ns_r + "/joint_commands",
      10);

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

  // Initialize the robot manager client
  if (!robot_manager->init())
  {
    printf("Failed to initialize the robot manager client\n");
    return -1;
  }
  // Blocking loop
  while (!robot_manager->robot_is_ready())
  {
    sleep(0.01);
  }
  // Start EGM mode
  if (!robot_manager->start_egm())
  {
    printf("Failed to start egm\n");
    return -1;
  }

  // Spin nodes to recieve joint states
  auto executor_r = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto executor_l = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor_r->add_node(node_s_r);
  executor_l->add_node(node_s_l);
  auto future_handle_r = std::async(std::launch::async, spin, executor_r);
  auto future_handle_l = std::async(std::launch::async, spin, executor_l);

  pose_estimation_manager = std::make_shared<ObjectPoseEstimator>("object_pose_estimator");

  pose_listener = std::make_shared<PoseListener>();

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

  bool cap_success{false};
  bool est_success{false};

  // Movements
  //--------------------------------------------------------------------------------------------------------------------

  // Go to home position (start pose)
  go_to_home_pos();


  while (1)
  {
    cap_success = pose_estimation_manager->call_capture_srv(10s);
    est_success = pose_estimation_manager->call_estimate_pose_srv(20s);
    if (est_success)
    {
      auto grasp_pose = pose_listener->get_graspable_chessboard_pose(0.05, true);

      std::array<double, 6> pose;
      std::copy_n(grasp_pose.begin(), 6, pose.begin());
      blocking_cart_p2p_motion_right(pose);
    }
  }
  return 0;
}
