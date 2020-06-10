// Copyright 2020 Markus Bjønnes and Marius Nilsen.
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
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sg_control_interfaces/action/grip.hpp>
#include <sg_control/visibility_control.h>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float32.hpp>

namespace sg_control
{
using Grip = sg_control_interfaces::action::Grip;
using GoalHandleGrip = rclcpp_action::ServerGoalHandle<Grip>;

class SgControl 
{
public:
  SG_CONTROL_PUBLIC
  SgControl(std::shared_ptr<rclcpp::Node> node);
  
  SG_CONTROL_PUBLIC
  bool init();
  
  SG_CONTROL_PUBLIC
  std::shared_ptr<rclcpp::Node> get_node(){ return node_; }

private:
  std::string namespace_;
  std::shared_ptr<rclcpp::Node> node_;

  rclcpp_action::Server<Grip>::SharedPtr grip_action_server_; 
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr jog_gripper_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gripper_position_publisher_;
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
  bool grip_in();
  bool grip_out();
  void jog_gripper(float pos);
  void jog_gripper_callback(std_msgs::msg::Float32::UniquePtr msg);
};

} //namespace sg_control

