#ifndef ROS_CONTROLLERS__JOINT_TRAJECTORY_CONTROLLER_HPP_
#define ROS_CONTROLLERS__JOINT_TRAJECTORY_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/operation_mode_handle.hpp"
#include "hardware_interface/robot_hardware.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <controllers/trajectory.hpp>
#include <controllers/visibility_control.h>
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <std_msgs/msg/bool.hpp>



namespace ros_controllers
{

class JointTrajectoryController : public controller_interface::ControllerInterface
{
public:

  ROS_CONTROLLERS_PUBLIC
  JointTrajectoryController();

  ROS_CONTROLLERS_PUBLIC
  JointTrajectoryController(const std::vector<std::string> & joint_names, 
                            const std::vector<std::string> & write_op_names);

  ROS_CONTROLLERS_PUBLIC
  controller_interface::controller_interface_ret_t
  init(std::weak_ptr<hardware_interface::RobotHardware> robot_hardware, const std::string & controller_name) override;

  ROS_CONTROLLERS_PUBLIC
  controller_interface::controller_interface_ret_t
  update() override;

  ROS_CONTROLLERS_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

  ROS_CONTROLLERS_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override;

  ROS_CONTROLLERS_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  ROS_CONTROLLERS_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  ROS_CONTROLLERS_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_error(const rclcpp_lifecycle::State & previous_state) override;

  ROS_CONTROLLERS_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

private:

  std::vector<std::string> joint_names_;
  std::vector<std::string> write_op_names_;

  std::vector<hardware_interface::JointCommandHandle *> registered_joint_cmd_handles_;
  std::vector<const hardware_interface::JointStateHandle *> registered_joint_state_handles_;
  std::vector<hardware_interface::OperationModeHandle *> registered_operation_mode_handles_;

  bool subscriber_is_active_ = false;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_command_subscriber_ = nullptr;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_command_subscriber_;

  TrajectoryPointConstIter prev_traj_point_ptr_;
  std::shared_ptr<Trajectory> * traj_point_active_ptr_ = nullptr;
  std::shared_ptr<Trajectory> traj_external_point_ptr_ = nullptr;
  std::shared_ptr<Trajectory> traj_home_point_ptr_ = nullptr;
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> traj_msg_home_ptr_ = nullptr;

  bool is_halted = false;
  bool is_stopped = false;
  bool new_trajectory = false;

  bool reset();
  void set_op_mode(const hardware_interface::OperationMode & mode);
  void halt();
  void stop_command_callback(std_msgs::msg::Bool::UniquePtr msg);
};

}  // namespace ros_controllers

#endif  // ROS_CONTROLLERS__JOINT_TRAJECTORY_CONTROLLER_HPP_