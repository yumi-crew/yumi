#include "external_force/external_force.hpp"

ExternalForce::ExternalForce(urdf::Model robot) : kdl_wrapper_(robot)
{
  kdl_wrapper_.init();

  int joints_r = kdl_wrapper_.get_right_arm().getNrOfJoints();
  int joints_l = kdl_wrapper_.get_left_arm().getNrOfJoints();
  q_l.resize(joints_l);
  q_r.resize(joints_r);
  q_dot_l.resize(joints_l);
  q_dot_r.resize(joints_r);
  torques_l.resize(joints_l);
  torques_r.resize(joints_r);

  joint_state_node_ = rclcpp::Node::make_shared("joint_state_node");
  joint_state_node_->create_subscription<sensor_msgs::msg::JointState>("/joint_states",
                                                                       10,
                                                                        std::bind());
}

