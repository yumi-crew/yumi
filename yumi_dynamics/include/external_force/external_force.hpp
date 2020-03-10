#pragma once


#include <urdf/model.h>
#include <kdl_wrapper/kdl_wrapper.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>


class ExternalForce
{
public:
  ExternalForce(urdf::Model);


private:
  KdlWrapper kdl_wrapper_;
  std::vector<float> q_l;
  std::vector<float> q_r;
  std::vector<float> q_dot_l;
  std::vector<float> q_dot_r;

  std::vector<float> torques_l;
  std::vector<float> torques_r;

  rclcpp::Subscription<sensor_msgs::msg::JointState> joint_state_sub_;
  //rclcpp::Subscription<> torque_sub_;

  void joint_state_callback(sensor_msgs::msg::JointState::UniquePtr jnt_msg);
  void torque_callback();
};


