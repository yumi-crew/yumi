#ifndef ROS_CONTROLLERS__JOINT_POSITION_CONTROLLER_HPP_
#define ROS_CONTROLLERS__JOINT_POSITION_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "controller_interface/controller_interface.hpp"

#include "hardware_interface/robot_hardware.hpp"

#include "rclcpp_lifecycle/state.hpp"

#include "controllers/visibility_control.h"

#include "sensor_msgs/msg/joint_state.hpp"

#include "ros2_control_interfaces/msg/joint_control.hpp"

// parameter server services
#include "parameter_server_interfaces/srv/get_controller_pid.hpp"
#include "parameter_server_interfaces/srv/get_controller_joints.hpp"

#include "ros2_control_utils/pid.hpp"


namespace ros_controllers
{

class JointPositionController : public controller_interface::ControllerInterface
{
public:
  ROS_CONTROLLERS_PUBLIC
  JointPositionController();

  ROS_CONTROLLERS_PUBLIC
  controller_interface::controller_interface_ret_t
  init( std::weak_ptr<hardware_interface::RobotHardware> robot_hardware,
        const std::string &controller_name ) override;

  ROS_CONTROLLERS_PUBLIC
  controller_interface::controller_interface_ret_t
  update() override;

  ROS_CONTROLLERS_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  ROS_CONTROLLERS_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  ROS_CONTROLLERS_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  ROS_CONTROLLERS_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  ROS_CONTROLLERS_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_error(const rclcpp_lifecycle::State &previous_state) override;

  ROS_CONTROLLERS_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

private:

  // Joint handles
  std::vector<hardware_interface::JointCommandHandle*> registered_joint_cmd_handles_ = {};
  std::vector<const hardware_interface::JointStateHandle*> registered_joint_state_handles_ = {};
  
  std::vector<std::shared_ptr<control_utils::Pid>> pid_controllers_ = {};

  std::unordered_map<std::string, double> desired_pos_map_ = {};
  std::unordered_map<std::string, std::shared_ptr<control_utils::Pid>> pid_controllers_map_ = {};


  rclcpp::Subscription<ros2_control_interfaces::msg::JointControl>::SharedPtr subscription_;

  rclcpp::Time previous_update_time_;

  // Configuration loading
  control_utils::Pid::Gains get_controller_pid();
  std::vector<std::string> get_controller_joints();

  // Callbacks
  void desired_position_subscrition_callback(ros2_control_interfaces::msg::JointControl::UniquePtr msg);

  // Nodegroup namespace
  std::string namespace_;
};


} // namespace ros_controllers

#endif // ROS_CONTROLLERS__JOINT_POSITION_CONTROLLER_HPP_