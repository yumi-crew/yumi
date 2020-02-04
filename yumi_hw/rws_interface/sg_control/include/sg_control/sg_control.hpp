#ifndef SG_CONTROl__SG_CONTROl_HPP_
#define SG_CONTROl__SG_CONTROl_HPP_

/*
 * Action server providing close and open actions on the SmartGrippers of YuMi.
 */

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>
#include <rclcpp_action/rclcpp_action.hpp>

#include <abb_librws/rws_rapid.h>
#include <abb_librws/rws_client.h>
#include <abb_librws/rws_interface.h>
#include <abb_librws/rws_state_machine_interface.h>

#include <sg_control_interfaces/action/grip.hpp>

#include <sg_control/visibility_control.h>



namespace sg_control
{

using Grip = sg_control_interfaces::action::Grip;
using GoalHandleGrip = rclcpp_action::ServerGoalHandle<Grip>;

class SgControl : public rclcpp::Node
{
public:

  SG_CONTROL_PUBLIC
  SgControl(rclcpp::NodeOptions &options, const std::string &ip);
  
  SG_CONTROL_PUBLIC
  bool init();

  SG_CONTROL_PUBLIC
  bool is_gripper_open();


private:

  std::string namespace_;

  // RWS
  std::string ip_;
  std::unique_ptr<abb::rws::RWSStateMachineInterface> rws_state_machine_interface_;
  std::unique_ptr<abb::rws::RWSStateMachineInterface::SGSettings> sg_settings_;


  // Action Server
  rclcpp_action::Server<Grip>::SharedPtr action_server_; 
  bool should_grip_in_;
  bool should_execute_ = false;
  
  rclcpp_action::GoalResponse 
  handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Grip::Goal> goal);

  rclcpp_action::CancelResponse 
  handle_cancel(const std::shared_ptr<GoalHandleGrip> goal_handle);

  void 
  handle_accepted(const std::shared_ptr<GoalHandleGrip> goal_handle);

  void 
  execute(const std::shared_ptr<GoalHandleGrip> goal_handle);


  // Action triggered gripper operations
  bool perform_grip_in();
  bool perform_grip_out();
 

  // Helper functions
  void busy_wait_for_gripper_to_finish_motion();
  bool grip_in();
  bool grip_out();

 
};

} //namespace sg_control


#endif  // SG_CONTROL__SG_CONTROL_HPP_