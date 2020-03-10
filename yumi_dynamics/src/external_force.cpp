#include "external_force/external_force.hpp"

namespace yumi_dynamics
{

ExternalForce::ExternalForce(urdf::Model robot) : kdl_wrapper_(robot)
{
  kdl_wrapper_.init();

  joints_r = kdl_wrapper_.get_right_arm().getNrOfJoints();
  joints_l = kdl_wrapper_.get_left_arm().getNrOfJoints();
  q_l.resize(joints_l);
  q_r.resize(joints_r);
  q_dot_l.resize(joints_l);
  q_dot_r.resize(joints_r);
  torques_l.resize(joints_l);
  torques_r.resize(joints_r);
  ext_torques_l.resize(joints_l);
  ext_torques_r.resize(joints_r);

  ext_torques_eig_l.resize(joints_l, 1);
  ext_torques_eig_r.resize(joints_r, 1);

  jacobian_l = KDL::Jacobian(joints_l);
  jacobian_r = KDL::Jacobian(joints_r);

  // joint_state_node_ = rclcpp::Node::make_shared("joint_state_node");
  ext_force_node_ = std::make_shared<rclcpp::Node>("ext_force");
  ext_force_node_->create_subscription<sensor_msgs::msg::JointState>("/joint_states",
                                                                     10,
                                                                     std::bind(&ExternalForce::joint_state_callback,
                                                                               this, std::placeholders::_1));
  ext_force_node_->create_subscription<sensor_msgs::msg::JointState>("/ext_joint_torques",
                                                                     10,
                                                                     std::bind(&ExternalForce::external_torques_callback,
                                                                               this, std::placeholders::_1));
  wrench_pub_ = ext_force_node_->create_publisher<geometry_msgs::msg::WrenchStamped>("/TCP_wrench", 10);
}

void ExternalForce::joint_state_callback(sensor_msgs::msg::JointState::UniquePtr jnt_msg)
{
  for (int i = 0; i < std::min(joints_r, joints_l); ++i)
  {
    q_l[i] = jnt_msg->position[i];
    q_r[i] = jnt_msg->position[joints_l + i];
    q_dot_l[i] = jnt_msg->velocity[i];
    q_dot_r[i] = jnt_msg->velocity[joints_l + i];
  }
}

void ExternalForce::external_torques_callback(sensor_msgs::msg::JointState::UniquePtr jnt_msg)
{
  for (int i = 0; i < std::min(joints_r, joints_l); ++i)
  {
    ext_torques_l[i] = jnt_msg->effort[i];
    ext_torques_r[i] = jnt_msg->effort[joints_l + i];

    ext_torques_eig_l(i, 1) = ext_torques_l[i];
    ext_torques_eig_r(i, 1) = ext_torques_r[i];
  }
}

void ExternalForce::torques_callback(sensor_msgs::msg::JointState::UniquePtr jnt_msg)
{
  for (int i = 0; i < std::min(joints_r, joints_l); ++i)
  {
    torques_l[i] = jnt_msg->effort[i];
    torques_r[i] = jnt_msg->effort[joints_l + i];
  }
}

void ExternalForce::estimate_TCP_wrench(geometry_msgs::msg::WrenchStamped &wrench_l, geometry_msgs::msg::WrenchStamped &wrench_r)
{
  jacobian_l = kdl_wrapper_.calculate_jacobian("right_arm", q_l);
  jacobian_r = kdl_wrapper_.calculate_jacobian("left_arm", q_r);
  // pseudoinverse (A.transpose()*A).inverse()*A.transpose() of the transposed jacobian matrices
  Eigen::MatrixXd jac_t_pinv_l =
      ((jacobian_l.data * jacobian_l.data.transpose()).inverse() * jacobian_l.data);
  Eigen::MatrixXd jac_t_pinv_r =
      ((jacobian_r.data * jacobian_r.data.transpose()).inverse() * jacobian_r.data);

  Eigen::VectorXd W_l = jac_t_pinv_l * ext_torques_eig_l;
  Eigen::VectorXd W_r = jac_t_pinv_r * ext_torques_eig_r;

  populate_wrench_msg(wrench_l, W_l);
  populate_wrench_msg(wrench_r, W_r);

  // //for testing without actual torque values
  // Eigen::Matrix<double, 7, 1> test_l;
  // Eigen::Matrix<double, 7, 1> test_r;
  // std::vector<double> test_torques = {1.0, 1.0, 1.0, 0.1, 0.5, 2.0, 0.3};
  // for (int i = 0; i < std::min(joints_r, joints_l); ++i)
  // {
  //   test_l(i, 1) = test_torques[i];
  //   test_r(i, 1) = test_torques[i];
  // }
}

void ExternalForce::populate_wrench_msg(geometry_msgs::msg::WrenchStamped &wrench_msg, Eigen::VectorXd &wrench)
{
  wrench_msg.wrench.force.x = wrench[0];
  wrench_msg.wrench.force.y = wrench[1];
  wrench_msg.wrench.force.z = wrench[2];
  wrench_msg.wrench.torque.x = wrench[3];
  wrench_msg.wrench.torque.y = wrench[4];
  wrench_msg.wrench.torque.z = wrench[5];
}

} // namespace yumi_dynamics