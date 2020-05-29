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

#include <rws_clients/gripper_client.hpp>

namespace rws_clients
{

GripperClient::GripperClient(std::shared_ptr<rclcpp::Node> node, std::string ns)
: 
node_(node), 
namespace_(ns)
{}

bool GripperClient::init()
{
  using std::placeholders::_1;

  // if global namespace, namespace_ becomes /, giving a invalid topic names. 
  // client should be gripper specific, global gripper clients are therfore not supported.
  if(namespace_.compare("/") == 0) { return false; }

  jog_gripper_publisher_ = node_->create_publisher<std_msgs::msg::Float32>(namespace_+"/jog_gripper", 10);

  action_client_ = rclcpp_action::create_client<Grip>(
    node_->get_node_base_interface(),
    node_->get_node_graph_interface(),
    node_->get_node_logging_interface(),
    node_->get_node_waitables_interface(),
    namespace_+"/Grip"
  );

  return true;
}


void GripperClient::send_goal(int percentage_closed)
{
  using namespace std::placeholders;

  if (!action_client_) 
  {
    RCLCPP_ERROR(node_->get_logger(), "Action client not initialized");
    return;
  }
  if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) 
  {
    RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
    return;
  }

  auto goal_msg = Grip::Goal();
  goal_msg.grip_percentage_closed = percentage_closed;

  RCLCPP_INFO(node_->get_logger(), "Sending goal");
  auto send_goal_options = rclcpp_action::Client<Grip>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(&GripperClient::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =  std::bind(&GripperClient::feedback_callback, this, _1, _2);
  send_goal_options.result_callback = std::bind(&GripperClient::result_callback, this, _1);
  auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);
}


void GripperClient::goal_response_callback(std::shared_future<GoalHandleGrip::SharedPtr> future)
{
  auto goal_handle = future.get();
  if (!goal_handle) 
  {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
  } 
  else 
  {
    RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
  }
}


void GripperClient::feedback_callback(GoalHandleGrip::SharedPtr, const std::shared_ptr<const Grip::Feedback> feedback)
{
}


void GripperClient::result_callback(const GoalHandleGrip::WrappedResult &result)
{
  switch (result.code) 
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;

    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
      return;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
      return;

    default:
      RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
      return;
  }

  RCLCPP_INFO(node_->get_logger(), "Result received, Gripper is %d percentage open", (int)result.result->res_grip);
}


void GripperClient::jog_gripper(double pos)
{
  std_msgs::msg::Float32 msg;
  msg.data = pos/2.0; // gripper = joint + mimic_joint
  jog_gripper_publisher_->publish(msg);
}

} //end namespace rws_clients