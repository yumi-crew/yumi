#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>
#include <unistd.h>
#include <fstream>
#include <math.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <angles/angles.h>

std::array<double, 8> recieved_joint_state_l{0, 0, 0, 0, 0, 0, 0, 0.02};
std::array<double, 8> recieved_joint_state_r{0, 0, 0, 0, 0, 0, 0, 0.02};

void signal_callback_handler(int signum)
{
  std::cout << "Caught signal " << signum << std::endl;
  // Terminate ros node
  rclcpp::shutdown();
  // Terminate program
  exit(signum);
}

sensor_msgs::msg::JointState combine_joint_states()
{
  sensor_msgs::msg::JointState combined;
  combined.name = {
      "yumi_joint_1_l", "yumi_joint_1_r", "yumi_joint_2_l", "yumi_joint_2_r",
      "yumi_joint_7_l", "yumi_joint_7_r", "yumi_joint_3_l", "yumi_joint_3_r",
      "yumi_joint_4_l", "yumi_joint_4_r", "yumi_joint_5_l", "yumi_joint_5_r",
      "yumi_joint_6_l", "yumi_joint_6_r", "gripper_l_joint", "gripper_r_joint",
      };

  for (int i = 0; i < 8; i++)
  {
    combined.position.push_back(recieved_joint_state_l[i]);
    combined.position.push_back(recieved_joint_state_r[i]);
  }
  return combined;
}

// Subscription callback
void callback_l(sensor_msgs::msg::JointState::UniquePtr msg)
{
  for (int i = 0; i < 7; i++)
  {
    recieved_joint_state_l[i] = msg->position[i];
  }
}

void callback_r(sensor_msgs::msg::JointState::UniquePtr msg)
{
  for (int i = 0; i < 7; i++)
  {
    recieved_joint_state_r[i] = msg->position[i];
  }
}

void spin(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exe)
{
  exe->spin();
}

int main(int argc, char *argv[])
{

  // Ctrl+C handler
  signal(SIGINT, signal_callback_handler);

  rclcpp::init(argc, argv);

  // making node
  auto node_s_r = rclcpp::Node::make_shared("joint_state_right_s");
  auto node_s_l = rclcpp::Node::make_shared("joint_state_left_s");
  auto node_p = rclcpp::Node::make_shared("joint_state_p");

  // creating publisher for global namespaced joint_space
  auto joint_command_publisher = node_p->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  // creating subscriptions to namespaced joint_states
  auto joint_state_subscription_r = node_s_r->create_subscription<sensor_msgs::msg::JointState>("/r/joint_states", 10, callback_r);
  auto joint_state_subscription_l = node_s_l->create_subscription<sensor_msgs::msg::JointState>("/l/joint_states", 10, callback_l);

  auto executor_r = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto executor_l = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor_r->add_node(node_s_r);
  executor_l->add_node(node_s_l);
  auto future_handle_r = std::async(std::launch::async, spin, executor_r);
  auto future_handle_l = std::async(std::launch::async, spin, executor_l);

  rclcpp::WallRate loop_rate(250);

  while (rclcpp::ok())
  {
    auto msg = combine_joint_states();
    joint_command_publisher->publish(msg);
    // Spin node to publish command
    rclcpp::spin_some(node_p);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
