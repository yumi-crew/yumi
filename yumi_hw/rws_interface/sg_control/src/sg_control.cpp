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

#include <sg_control/sg_control.hpp>

namespace sg_control
{

SgControl::SgControl(rclcpp::NodeOptions &options, const std::string &ip)
    : Node("sg_control", options),
      ip_(ip)
{
}

bool SgControl::init()
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  // Using nodegroup namespace to determine which of the grippers this instance is representing
  namespace_ = this->get_namespace();

  sg_settings_ = std::make_shared<abb::rws::RWSStateMachineInterface::SGSettings>();
  rws_state_machine_interface_ = std::make_shared<abb::rws::RWSStateMachineInterface>(ip_);
  gripper_position_publisher_ = this->create_publisher<std_msgs::msg::Float64>(namespace_ + "/gripper_pos", 10);
  jog_gripper_sub_ = this->create_subscription<std_msgs::msg::Float32>(namespace_+"/jog_gripper", 10,
   std::bind(&SgControl::jog_gripper_callback, this, _1));

  // Connection check. Confirm robot controller is connected. Loops until connection is made.
  auto runtime_info = rws_state_machine_interface_->collectRuntimeInfo();
  RCLCPP_INFO(this->get_logger(), "Connecting to Robot...");
  if (!runtime_info.rws_connected)
  {
    RCLCPP_ERROR(this->get_logger(), "Connection failed. Check robot is connected.");
    return false;
  }

  // Start action server
  action_server_ = rclcpp_action::create_server<Grip>(
      this->shared_from_this(),
      "Grip",
      std::bind(&SgControl::handle_goal, this, _1, _2),
      std::bind(&SgControl::handle_cancel, this, _1),
      std::bind(&SgControl::handle_accepted, this, _1));
  return true;
}

rclcpp_action::GoalResponse
SgControl::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Grip::Goal> goal)
{
  switch ((int)goal->grip_percentage_closed)
  {
  case 0:
    should_grip_in_ = false;
    RCLCPP_INFO(this->get_logger(), "Recieved goal request: Grip Out");
    break;
  case 100:
    should_grip_in_ = true;
    RCLCPP_INFO(this->get_logger(), "Recieved goal request: Grip In");
    break;
  default:
    RCLCPP_ERROR(this->get_logger(), "Only values 0 and 100 is supported at the moment");
  }
  (void)uuid;
  should_execute_ = true;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
SgControl::handle_cancel(const std::shared_ptr<GoalHandleGrip> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Not possible to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SgControl::handle_accepted(const std::shared_ptr<GoalHandleGrip> goal_handle)
{
  using std::placeholders::_1;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&SgControl::execute, this, _1), goal_handle}.detach();
}

void SgControl::execute(const std::shared_ptr<GoalHandleGrip> goal_handle)
{
  if (should_grip_in_ && should_execute_)
  {
    perform_grip_in();
    RCLCPP_INFO(this->get_logger(), "Executing Grip In");
  }
  if (!should_grip_in_ && should_execute_)
  {
    perform_grip_out();
    RCLCPP_INFO(this->get_logger(), "Executing Grip Out");
  }

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<Grip::Feedback>();
  auto &position = feedback->position;
  auto result = std::make_shared<Grip::Result>();

  // Estimated to take a second to close gripper. Publish feedback at 250 hz for a second.
  auto start_time = this->now();
  auto elapsed_time = this->now() - start_time;
  double percentage = 0;
  rclcpp::Rate loop(250);
  while (elapsed_time.seconds() < 1.0)
  {
    // Check if there is a cancel request
    if (goal_handle->is_canceling())
    {
      result->res_grip = position;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal Canceled");
      should_execute_ = false;
      return;
    }
    // Find gripper pos and publish feedback
    std::string s_pos = get_gripper_pos();
    position = std::stof(s_pos); 
    goal_handle->publish_feedback(feedback);
    loop.sleep();
    elapsed_time = this->now() - start_time;
  }

  // Will not evaluate if gripper is closed/opened. Even though the desired
  // action was to grip in, the gripper may grasp an object. 
  if (should_grip_in_) percentage = 0;
  else percentage = 100;

  result->res_grip = percentage; //percentage closed
  goal_handle->succeed(result);
  RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
  should_execute_ = false;
}

bool SgControl::perform_grip_in()
{
  // RAPID execution check. Confirm StateMachine is running.
  if (!rws_state_machine_interface_->isRAPIDRunning().isTrue())
  {
    RCLCPP_ERROR(this->get_logger(), "Unable to grip in without StateMachine executing.");
    return false;
  }
  // grips in with correct gripper
  grip_in();
}

bool SgControl::perform_grip_out()
{
  // RAPID execution check. Confirm StateMachine is running.
  if (!rws_state_machine_interface_->isRAPIDRunning().isTrue())
  {
    RCLCPP_ERROR(this->get_logger(), "Unable to grip out without StateMachine executing.");
    return false;
  }
  // grips out with correct gripper
  grip_out();
}

bool SgControl::grip_in()
{
  if (namespace_.compare("/r") == 0)
  {
    if (!rws_state_machine_interface_->services().sg().rightGripIn())
    {
      RCLCPP_ERROR(this->get_logger(), "Unable to grip in with right gripper.");
      return false;
    }
    else
      return true;
  }
  else if (namespace_.compare("/l") == 0)
  {
    if (!rws_state_machine_interface_->services().sg().leftGripIn())
    {
      RCLCPP_ERROR(this->get_logger(), "Unable to grip in with left gripper.");
      return false;
    }
    else
      return true;
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid namepsace occured.");
    return false;
  }
}

bool SgControl::grip_out()
{
  if (namespace_.compare("/r") == 0)
  {
    if (!rws_state_machine_interface_->services().sg().rightGripOut())
    {
      RCLCPP_ERROR(this->get_logger(), "Unable to grip out with right gripper.");
      return false;
    }
    else
      return true;
  }
  else if (namespace_.compare("/l") == 0)
  {
    if (!rws_state_machine_interface_->services().sg().leftGripOut())
    {
      RCLCPP_ERROR(this->get_logger(), "Unable to grip out with left gripper.");
      return false;
    }
    else
      return true;
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid namespace occured.");
    return false;
  }
}

std::string SgControl::get_gripper_pos()
{
  if (namespace_.compare("/l") == 0)
  {
    return rws_state_machine_interface_->getIOSignal("hand_ActualPosition_L");
  }
  else if (namespace_.compare("/r") == 0)
  {
    return rws_state_machine_interface_->getIOSignal("hand_ActualPosition_R");
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid namespace occured.");
    return {};
  }
}

void SgControl::publish_gripper_position()
{
  std::string s_pos = get_gripper_pos();
  if (!s_pos.empty())
  {
    std_msgs::msg::Float64 msg;
    msg.data = std::stod(s_pos)/10000.0;
    gripper_position_publisher_->publish(msg);
  }
}


void SgControl::jog_gripper_callback(std_msgs::msg::Float32::UniquePtr msg)
{
  jog_gripper(msg->data);
}


void SgControl::jog_gripper(float pos)
{
  if (namespace_.compare("/l") == 0)
  {
    rws_state_machine_interface_->services().sg().leftMoveTo(pos);
  }
  else if (namespace_.compare("/r") == 0)
  {
    rws_state_machine_interface_->services().sg().rightMoveTo(pos);
  }
}

} //end namespace sg_control
