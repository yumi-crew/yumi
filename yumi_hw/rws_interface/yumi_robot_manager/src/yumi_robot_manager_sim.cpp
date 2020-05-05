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

#include <yumi_robot_manager/yumi_robot_manager_sim.h>

namespace yumi_robot_manager
{

YumiRobotManager::YumiRobotManager(const std::string &name, const std::string &ip_address) 
: 
name_(name)
{}


bool YumiRobotManager::init()
{
  node_ = rclcpp::Node::make_shared(name_); 

  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;

  stop_egm_srv_ = node_->create_service<StopEgm>(
    "StopEgm", 
    std::bind(&YumiRobotManager::handle_StopEgm, this, _1, _2, _3), 
    rmw_qos_profile_services_default
  );

  start_egm_srv_ = node_->create_service<StartEgm>(
    "StartEgm", 
    std::bind(&YumiRobotManager::handle_StartEgm, this, _1, _2, _3), 
    rmw_qos_profile_services_default
  );

  is_ready_srv_ = node_->create_service<IsReady>(
    "IsReady", 
    std::bind(&YumiRobotManager::handle_IsReady, this, _1, _2, _3), 
    rmw_qos_profile_services_default
  );

  stop_motors_srv_ = node_->create_service<StopMotors>(
  "StopMotors",
  std::bind(&YumiRobotManager::handle_StopMotors, this, _1, _2, _3),
  rmw_qos_profile_services_default
  );

  return true;
}


bool YumiRobotManager::start_state_machine()
{
  return true;
}


bool YumiRobotManager::go_to_state(std::string mode)
{
  return true;
}


bool YumiRobotManager::configure()
{
  return true;
}


void YumiRobotManager::spin()
{
  rclcpp::spin(node_);
}


//----------Helper Functions--------------------------------------------------------------------------------------------
bool YumiRobotManager::configure_egm()
{
  return true;
}


void YumiRobotManager::busy_wait_until_idle()
{
  sleep(1);
}


void YumiRobotManager::wait_for_gripper_to_finish_motion()
{
  // Assumption: Worst Case Execution Time < 2s
  sleep(2);
}

bool YumiRobotManager::calibrate_grippers()
{
  return true;
}

bool YumiRobotManager::stop_egm()
{
  return true;
}

//---------- Service server handler functions---------------------------------------------------------------------------
void YumiRobotManager::handle_StopEgm(const std::shared_ptr<rmw_request_id_t> request_header,
                                      const std::shared_ptr<StopEgm::Request> request,
                                      const std::shared_ptr<StopEgm::Response> response)
{
  (void) request_header;
  (void) request;
  if(stop_egm())
  {
    response->is_stopped = true;
  }
  else
  {
    response->is_stopped = false;
  }
}


void YumiRobotManager::handle_StartEgm(const std::shared_ptr<rmw_request_id_t> request_header,
                                       const std::shared_ptr<StartEgm::Request> request,
                                       const std::shared_ptr<StartEgm::Response> response)
{   
  (void) request_header;
  (void) request;
  if(go_to_state("egm"))
  {
    response->is_started = true;
  }
  else
  {
    response->is_started = false;
  } 
}


void YumiRobotManager::handle_IsReady(const std::shared_ptr<rmw_request_id_t> request_header,
                                      const std::shared_ptr<IsReady::Request> request,
                                      const std::shared_ptr<IsReady::Response> response)
{   
  (void) request_header;
  (void) request;
  response->is_ready = true;
}


void YumiRobotManager::handle_StopMotors(const std::shared_ptr<rmw_request_id_t> request_header,
                                         const std::shared_ptr<StopMotors::Request> request,
                                         const std::shared_ptr<StopMotors::Response> response)
{
  (void) request_header;
  (void) request;
  response->motors_off = true;
}

} // namespace yumi_robot_manager