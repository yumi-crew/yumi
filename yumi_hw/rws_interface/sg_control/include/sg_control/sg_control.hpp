// Copyright 2020 Norwegian University of Science and Technology.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <string>
#include <memory>
#include <math.h>  
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float32.hpp>
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
  void publish_gripper_position();

private:
  std::string namespace_;
  std::string ip_;
  std::shared_ptr<abb::rws::RWSStateMachineInterface> rws_state_machine_interface_;
  std::shared_ptr<abb::rws::RWSStateMachineInterface::SGSettings> sg_settings_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> gripper_position_publisher_;
  rclcpp_action::Server<Grip>::SharedPtr action_server_; 

  bool should_grip_in_;
  bool should_execute_ = false;
  double allowed_deviation_ = 0.001;
  
  rclcpp_action::GoalResponse 
  handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Grip::Goal> goal);

  rclcpp_action::CancelResponse 
  handle_cancel(const std::shared_ptr<GoalHandleGrip> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleGrip> goal_handle);
  void execute(const std::shared_ptr<GoalHandleGrip> goal_handle);

  // Action triggered gripper operations
  bool perform_grip_in();
  bool perform_grip_out();
 
  // Helper functions
  std::string get_gripper_pos();
  bool grip_in();
  bool grip_out();
};

} //namespace sg_control

