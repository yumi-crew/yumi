#pragma once

#include <string>
#include <vector>

#include <urdf/model.h>
#include <kdl_wrapper/kdl_wrapper.h>
#include <eigen3/Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

namespace yumi_dynamics
{
class ExternalForce
{
public:
  ExternalForce(urdf::Model);
  void estimate_TCP_wrench();

private:
  KdlWrapper kdl_wrapper_;
  std::vector<float> q_l;
  std::vector<float> q_r;
  std::vector<float> q_dot_l;
  std::vector<float> q_dot_r;

  std::vector<float> torques_l;
  std::vector<float> torques_r;
  std::vector<float> ext_torques_l;
  std::vector<float> ext_torques_r;

  Eigen::MatrixXd ext_torques_eig_l;
  Eigen::MatrixXd ext_torques_eig_r;

  int joints_r;
  int joints_l;

  KDL::Jacobian jacobian_l;
  KDL::Jacobian jacobian_r;

  std::shared_ptr<rclcpp::Node> ext_force_node_;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>> wrench_pub_r_;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>> wrench_pub_l_;

  void joint_state_callback(sensor_msgs::msg::JointState::UniquePtr jnt_msg);
  void external_torques_callback(sensor_msgs::msg::JointState::UniquePtr jnt_msg);
  void torques_callback(sensor_msgs::msg::JointState::UniquePtr jnt_msg);
  void populate_wrench_msg(std::string mech_unit, geometry_msgs::msg::WrenchStamped &wrench_msg, Eigen::Matrix<double, 6, 1> &wrench);
};
} // namespace yumi_dynamics
