#include "external_force/external_force.hpp"
#include <fstream>

namespace yumi_dynamics
{
  ExternalForce::ExternalForce(urdf::Model robot) : kdl_wrapper_(robot)
  {
    kdl_wrapper_.init();

    joints_r_ = kdl_wrapper_.get_right_arm().getNrOfJoints();
    joints_l_ = kdl_wrapper_.get_left_arm().getNrOfJoints();

    q_l_.resize(joints_l_);
    q_r_.resize(joints_r_);
    q_dot_l_.resize(joints_l_);
    q_dot_r_.resize(joints_r_);

    torques_l_.resize(joints_l_, 1);
    torques_r_.resize(joints_r_, 1);
    ext_torques_l_.resize(joints_l_, 1);
    ext_torques_r_.resize(joints_r_, 1);

    jacobian_l_ = KDL::Jacobian(joints_l_);
    jacobian_r_ = KDL::Jacobian(joints_r_);

    ext_force_node_ = std::make_shared<rclcpp::Node>("ext_force");
    joint_state_sub_ = ext_force_node_->create_subscription<sensor_msgs::msg::JointState>("/joint_states",
                                                                                          10,
                                                                                          std::bind(&ExternalForce::joint_state_callback,
                                                                                                    this, std::placeholders::_1));
    ext_torque_sub_ = ext_force_node_->create_subscription<sensor_msgs::msg::JointState>("/r/external_joint_torques",
                                                                                         10,
                                                                                         std::bind(&ExternalForce::external_torques_callback,
                                                                                                   this, std::placeholders::_1));
    //wrench_pub_l_ = ext_force_node_->create_publisher<geometry_msgs::msg::WrenchStamped>("/l/TCP_wrench", 10);
    wrench_pub_r_ = ext_force_node_->create_publisher<geometry_msgs::msg::WrenchStamped>("/r/TCP_wrench", 10);
    exec_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    exec_->add_node(ext_force_node_);

    async_spinner_ = std::make_shared<AsyncSpinner>(exec_);
    async_spinner_->async_spin();
  }

  void ExternalForce::joint_state_callback(sensor_msgs::msg::JointState::UniquePtr jnt_msg)
  {
    for (int i = 0; i < std::min(joints_r_, joints_l_); ++i)
    {
      q_l_[i] = jnt_msg->position[i];
      q_r_[i] = jnt_msg->position[joints_l_ + i];
      q_dot_l_[i] = jnt_msg->velocity[i];
      q_dot_r_[i] = jnt_msg->velocity[joints_l_ + i];
    }
    if (!jnt_state_callback_ok_)
      jnt_state_callback_ok_ = true;
  }

  void ExternalForce::external_torques_callback(sensor_msgs::msg::JointState::UniquePtr jnt_msg)
  {
    for (int i = 0; i < std::min(joints_r_, joints_l_); ++i)
    {
      ext_torques_r_(i) = jnt_msg->effort[i];
      // ext_torques_r_(i) = jnt_msg->effort[joints_l_ + i];
    }
    if (!ext_trq_callback_ok_)
      ext_trq_callback_ok_ = true;
  }

  void ExternalForce::torques_callback(sensor_msgs::msg::JointState::UniquePtr jnt_msg)
  {
    // for (int i = 0; i < std::min(joints_r_, joints_l_); ++i)
    // {
    //   torques_l_(i) = jnt_msg->effort[i];
    //   torques_r_(i) = jnt_msg->effort[joints_l_ + i];
    // }
    // if (!trq_callback_ok_)
    //   trq_callback_ok_ = true;
  }

  void ExternalForce::estimate_TCP_wrench()
  {
    if (!jnt_state_callback_ok_ && !ext_trq_callback_ok_)
    {
      return;
    }
    try
    {
      //jacobian_l_ = kdl_wrapper_.calculate_jacobian("left_arm", q_l_);
      jacobian_r_ = kdl_wrapper_.calculate_jacobian("right_arm", q_r_);
    }
    catch (const std::exception &e)
    {
      std::cout << e.what() << std::endl;
    }

    // pseudoinverse (A.transpose()*A).inverse()*A.transpose() of the transposed jacobian matrices
    // Eigen::MatrixXd jac_t_pinv_l =
    //     ((jacobian_l_.data * jacobian_l_.data.transpose()).inverse() * jacobian_l_.data);
    Eigen::MatrixXd jac_t_pinv_r =
        ((jacobian_r_.data * jacobian_r_.data.transpose()).inverse() * jacobian_r_.data);

    //for testing without actual torque values
    // std::vector<double> test_torques = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // for (int i = 0; i < std::min(joints_r_, joints_l_); ++i)
    // {
    //   ext_torques_l_(i) = test_torques[i];
    //   ext_torques_r_(i) = test_torques[i];
    // }
    // end test

    // gravity compansation
    // ext_torques_l_ -= kdl_wrapper_.dynamics_gravity("left_arm", q_l_).data;
    // ext_torques_r_ -= kdl_wrapper_.dynamics_gravity("right_arm", q_r_).data;

    // calculate the TCP wrenches
    //Eigen::Matrix<double, 6, 1> W_l = jac_t_pinv_l * ext_torques_l_;
    Eigen::Matrix<double, 6, 1> W_r = jac_t_pinv_r * ext_torques_r_;

    geometry_msgs::msg::WrenchStamped wrench_l, wrench_r;

    //populate_wrench_msg("TCP_left_arm", wrench_l, W_l);
    populate_wrench_msg("TCP_right_arm", wrench_r, W_r);

    std::fstream log{"force_log.txt", std::fstream::app};
    if (log)
    {
      log << "x:" << wrench_r.wrench.force.x << ",y:" << wrench_r.wrench.force.y << ",z:" << wrench_r.wrench.force.z << std::endl;
      log.close();
    }
    std::fstream log2{"torque_log.txt", std::fstream::app};
    if (log2)
    {
      for(int i =0;i<joints_r_;++i)
      {
        log2 << ext_torques_r_(i) << ",";
      }
      log2 << std::endl;
      log2.close();
    }
 

    //wrench_pub_l_->publish(wrench_l);
    wrench_pub_r_->publish(wrench_r);
  }

  void ExternalForce::populate_wrench_msg(std::string mech_unit, geometry_msgs::msg::WrenchStamped &wrench_msg, Eigen::Matrix<double, 6, 1> &wrench)
  {
    wrench_msg.header.frame_id = mech_unit;
    wrench_msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();

    wrench_msg.wrench.force.x = wrench[0];
    wrench_msg.wrench.force.y = wrench[1];
    wrench_msg.wrench.force.z = wrench[2];
    wrench_msg.wrench.torque.x = wrench[3];
    wrench_msg.wrench.torque.y = wrench[4];
    wrench_msg.wrench.torque.z = wrench[5];
  }

} // namespace yumi_dynamics