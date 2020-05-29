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

#include <sg_control/sg_control_sim.h>

namespace sg_control
{

SgControl::SgControl(std::shared_ptr<rclcpp::Node> node)
:
node_(node)
{}


bool SgControl::init()
{
  using std::placeholders::_1; using std::placeholders::_2;

  // Using nodegroup namespace to determine which of the grippers this instance is representing
  namespace_ = node_->get_namespace();
   
  // Start action server
  grip_action_server_ = rclcpp_action::create_server<Grip>(       
    node_->shared_from_this(),
    "Grip",    // Topic names related
    std::bind(&SgControl::handle_goal, this, _1, _2), 
    std::bind(&SgControl::handle_cancel, this, _1),
    std::bind(&SgControl::handle_accepted, this, _1)
  );
  return true;
}


rclcpp_action::GoalResponse SgControl::handle_goal(const rclcpp_action::GoalUUID &uuid, 
                                                   std::shared_ptr<const Grip::Goal> goal)
{
  switch((int)goal->grip_percentage_closed)
  {
    case 0:
      should_grip_in_ = false;
      RCLCPP_INFO(node_->get_logger(), "Recieved goal request: Grip Out");
      break;

    case 100:
      should_grip_in_ = true;
      RCLCPP_INFO(node_->get_logger(), "Recieved goal request: Grip In");
      break;

    default:
      RCLCPP_ERROR(node_->get_logger(), "Only values 0 and 100 is supported at the moment");
  }
 
  (void)uuid;
  should_execute_ = true;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


rclcpp_action::CancelResponse SgControl::handle_cancel(const std::shared_ptr<GoalHandleGrip> goal_handle)
{
  RCLCPP_WARN(node_->get_logger(), "Not possible to cancel goal.");
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
  if(should_grip_in_ && should_execute_)
  {
    perform_grip_in();
    RCLCPP_INFO(node_->get_logger(), "Executing Grip In");
  }
  if(!should_grip_in_ && should_execute_)
  {
    perform_grip_out();
    RCLCPP_INFO(node_->get_logger(), "Executing Grip Out");
  }

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<Grip::Feedback>();
  auto &position = feedback->position;
  auto result = std::make_shared<Grip::Result>();
  
  // Estimated to take a second to close gripper. Publish feedback at 250 hz.
  auto start_time = node_->now();
  auto elapsed_time = node_->now()-start_time;
  double percentage = 0;
  rclcpp::Rate loop(250);
  while(elapsed_time.seconds() < 1.0)
  {
    // Check if there is a cancel request
    if (goal_handle->is_canceling()) 
    {
      result->res_grip = position;
      goal_handle->canceled(result);
      RCLCPP_INFO(node_->get_logger(), "Goal Canceled");
      should_execute_ = false;
      return;
    }
    // Estimate gripper position.
    elapsed_time = node_->now()-start_time;
    if(should_grip_in_) position = 0.02 - (0.02/1.0)*elapsed_time.seconds();
    else position = (0.02/1.0)*elapsed_time.seconds();
    
    // Publish feedback
    goal_handle->publish_feedback(feedback);
    loop.sleep();
    elapsed_time = node_->now()-start_time;
  }

  if(should_grip_in_) percentage = ceil((position/0.02)*100);
  else percentage = floor((position/0.02)*100); 
    
  if(percentage < 3) percentage = 0;
  if(percentage > 97) percentage = 100;

  // Check if goal is done
  if( (should_grip_in_ && percentage==100)  || (!should_grip_in_ && percentage==0))
  {
    result->res_grip = percentage; //percentage closed
    goal_handle->succeed(result);
    RCLCPP_INFO(node_->get_logger(), "Goal Succeeded");
    should_execute_ = false;
  }
}


bool SgControl::perform_grip_in()
{
  return grip_in();
}

bool SgControl::perform_grip_out()
{
  return grip_out();
}

bool SgControl::grip_in()
{
  return true;
}

bool SgControl::grip_out()
{
  return true;
}

} //end namespace sg_control