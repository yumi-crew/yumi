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

#include <memory>
#include <inttypes.h>
#include <string>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rws_clients/visibility_control.h>
#include <sg_control_interfaces/action/grip.hpp>

namespace rws_clients
{
using Grip = sg_control_interfaces::action::Grip;
using GoalHandleGrip = rclcpp_action::ClientGoalHandle<Grip>;

class GripClient : public rclcpp::Node
{
public:
  RWS_CLIENTS_PUBLIC
  GripClient(std::string name, std::string ns);

  RWS_CLIENTS_PUBLIC
  bool init();
  
  RWS_CLIENTS_PUBLIC
  bool is_goal_done() const;
 
  RWS_CLIENTS_PUBLIC
  void perform_grip(int percentage_closed);
   
private:
  rclcpp_action::Client<Grip>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_ = false;
  std::string namespace_;
  void send_goal();
  int percentage_closed_;

  void goal_response_callback(std::shared_future<GoalHandleGrip::SharedPtr> future);
  void feedback_callback(GoalHandleGrip::SharedPtr,  const std::shared_ptr<const Grip::Feedback> feedback);
  void result_callback(const GoalHandleGrip::WrappedResult &result);
};

}   // end namespace rws_clients


