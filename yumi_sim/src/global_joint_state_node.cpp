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
#include <sg_control_interfaces/action/grip.hpp>
#include <angles/angles.h>

std::array<double, 8> recieved_joint_pos_l{0, 0, 0, 0, 0, 0, 0, 0.02};
std::array<double, 8> recieved_joint_pos_r{0, 0, 0, 0, 0, 0, 0, 0.02};
std::array<double, 8> recieved_joint_vel_l{0, 0, 0, 0, 0, 0, 0, 0};
std::array<double, 8> recieved_joint_vel_r{0, 0, 0, 0, 0, 0, 0, 0};

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
  combined.header.stamp = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)->now();
  combined.name = {
      "yumi_joint_1_l",
      "yumi_joint_2_l",
      "yumi_joint_7_l",
      "yumi_joint_3_l",
      "yumi_joint_4_l",
      "yumi_joint_5_l",
      "yumi_joint_6_l",
      "yumi_joint_1_r",
      "yumi_joint_2_r",
      "yumi_joint_7_r",
      "yumi_joint_3_r",
      "yumi_joint_4_r",
      "yumi_joint_5_r",
      "yumi_joint_6_r",
      "gripper_l_joint",
      "gripper_r_joint",
  };

  for (int i = 0; i < 7; i++)
  {
    combined.position.push_back(recieved_joint_pos_l[i]);
    combined.velocity.push_back(recieved_joint_vel_l[i]);
  }
  for (int i = 0; i < 7; i++)
  {
    combined.position.push_back(recieved_joint_pos_r[i]);
    combined.velocity.push_back(recieved_joint_vel_r[i]);
  }

  combined.position.push_back(recieved_joint_pos_l[7]);
  combined.position.push_back(recieved_joint_pos_r[7]);
  combined.velocity.push_back(recieved_joint_vel_l[7]);
  combined.velocity.push_back(recieved_joint_vel_r[7]);
  return combined;
}

void callback_l(sensor_msgs::msg::JointState::UniquePtr msg)
{
  for (int i = 0; i < 7; i++)
  {
    recieved_joint_pos_l[i] = msg->position[i];
    recieved_joint_vel_l[i] = msg->velocity[i];
  }
}

void callback_r(sensor_msgs::msg::JointState::UniquePtr msg)
{
  for (int i = 0; i < 7; i++)
  {
    recieved_joint_pos_r[i] = msg->position[i];
    recieved_joint_vel_r[i] = msg->velocity[i];
  }
}

void callback_g_r(sg_control_interfaces::action::Grip_FeedbackMessage::UniquePtr msg)
{
  recieved_joint_pos_r[7] = msg->feedback.position;
}

void callback_g_l(sg_control_interfaces::action::Grip_FeedbackMessage::UniquePtr msg)
{
  recieved_joint_pos_l[7] = msg->feedback.position;
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
  auto node = rclcpp::Node::make_shared("joint_states_combinder");

  // publisher for global namespaced joint_space
  auto joint_command_publisher = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  // subscriptions to namespaced joint_states
  auto joint_state_subscription_r = node->create_subscription<sensor_msgs::msg::JointState>("/r/joint_states", 10, callback_r);
  auto joint_state_subscription_l = node->create_subscription<sensor_msgs::msg::JointState>("/l/joint_states", 10, callback_l);
  // gripper states
  auto gripper_state_subscription_r = node->create_subscription<sg_control_interfaces::action::Grip_FeedbackMessage>("/r/Grip/_action/feedback",10, callback_g_r);
  auto gripper_state_subscription_l = node->create_subscription<sg_control_interfaces::action::Grip_FeedbackMessage>("/l/Grip/_action/feedback",10, callback_g_l);

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  auto future_handle = std::async(std::launch::async, spin, executor);
  
  rclcpp::WallRate loop_rate(250);
  while (rclcpp::ok())
  {
    auto msg = combine_joint_states();
    joint_command_publisher->publish(msg);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
