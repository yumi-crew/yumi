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

#include <yumi_robot_manager/yumi_robot_manager.hpp>

namespace yumi_robot_manager
{

YumiRobotManager::YumiRobotManager(const std::string &name, const std::string &ip_address) 
: 
name_(name),
abb::rws::RWSStateMachineInterface(ip_address)
{}


bool YumiRobotManager::init()
{
  //--------------------------------------------------------------------------------------------------------------------
  // Setup. Performs checking of StateMachine's required conditions. StateMachine is verified to not be running
  // Upon completion. Motor state not modified.
  //--------------------------------------------------------------------------------------------------------------------
  
  node_ = rclcpp::Node::make_shared(name_); 
  egm_settings_l_ = std::make_shared<abb::rws::RWSStateMachineInterface::EGMSettings>();
  egm_settings_r_ = std::make_shared<abb::rws::RWSStateMachineInterface::EGMSettings>();

  // Connection check. Checks if robot controller is connected. Loops until connection is made.
  auto runtime_info = this->collectRuntimeInfo();
  RCLCPP_INFO(node_->get_logger(), "Connecting to Robot...");
  while(!runtime_info.rws_connected)
  {
    RCLCPP_WARN(node_->get_logger(), "Connection failed. Check robot is connected. Retrying...");
    runtime_info = this->collectRuntimeInfo();
    usleep(2*1000000);
  }
  RCLCPP_INFO(node_->get_logger(), "Succesfully connected to Robot.");

  // Auto mode check. Check if robot controller is in Auto mode. Auto mode is required for RWS.
  RCLCPP_INFO(node_->get_logger(), "Checking if robot controller is in auto mode...");
  if(!this->isAutoMode().isTrue() )
  {
    RCLCPP_ERROR(node_->get_logger(), "Robot controller must be in auto mode. Exiting.");
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Robot controller confirmed in auto mode.");

  // RAPID Execution check. Checks if RAPID Execution is stopped so program pointer can be reset. Additonally to
  // avoid unexpected execution during configuration the StateMachine should not be running.
  RCLCPP_INFO(node_->get_logger(), "Checking if StateMachine is running...");
  if(this->isRAPIDRunning().isTrue())
  {
    RCLCPP_WARN(node_->get_logger(), "StateMachine should not be running during initialization. Stopping StateMachine...");
    this->stopRAPIDExecution();
    sleep(1);
    if(this->isRAPIDRunning().isTrue())
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to stop StateMachine");
      return false;
    }
  }
  RCLCPP_INFO(node_->get_logger(), "StateMachine confirmed not running.");

  // Setting up services
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
  // Motor check. Check if motors are on, turn on if not.
  RCLCPP_INFO(node_->get_logger(), "Checking if motors are on...");
  if(!this->isMotorOn().isTrue())
  {
    if(!this->setMotorsOn())
    {
      RCLCPP_WARN(node_->get_logger(), "Not able to turn on motors");
    }
  }
  RCLCPP_INFO(node_->get_logger(), "Motors are on.");  

  // Reset program pointer.
  if(!first_execution_)
  {
    RCLCPP_INFO(node_->get_logger(), "Reseting program pointer...");       
    if(!this->resetRAPIDProgramPointer())
    {
      RCLCPP_ERROR(node_->get_logger(), "Not able to reset program pointer. Exiting");
      return false;
    }
    RCLCPP_INFO(node_->get_logger(), "Program pointer succesfully reset.");  
  }
  else
  {
    // To fix documented issue with abb_libegm
    this->startRAPIDExecution();
    usleep(0.5*1000000);
    this->stopRAPIDExecution();
    usleep(0.5*1000000);
    this->resetRAPIDProgramPointer();
  }


  // Start state machine.
  RCLCPP_INFO(node_->get_logger(), "Starting StateMachine...");
  this->startRAPIDExecution();
  if(!this->isRAPIDRunning().isTrue())
  {
    RCLCPP_WARN(node_->get_logger(), "Unable to start StateMachine");
    return false;
  }  
  RCLCPP_INFO(node_->get_logger(), "StateMachine started."); 

  // Give StateMachine time to initialize.
  usleep(1.5*100000);

  // To ensure true is not returned before StateMachine is in STATE_IDLE.
  busy_wait_until_idle();
  first_execution_ = false;
  return true;
}


bool YumiRobotManager::go_to_state(std::string mode)
{
  requested_state_ = boost::algorithm::to_lower_copy(mode);

  // RAPID execution check. Check if StateMachine is running.
  if(!this->isRAPIDRunning().isTrue())
  {
    RCLCPP_ERROR(node_->get_logger(), "Unable to change state, please confirm StateMachine is running");
    return false;
  }

  // StateMachine need to finish initializing and enter STATE_IDLE for EGM signals to be handled.
  busy_wait_until_idle();

  // STATE_RUN_EGM_ROUTINE
  if(requested_state_ == "egm")
  {
    // Sending signal EGM_START_JOINT triggers a interrupt which transitions StateMachine to STATE_RUN_EGM_ROUTINE 
    // and sets ACTION_RUN_JOINT before returning to its main and starting runEGMRoutine()
    RCLCPP_INFO(node_->get_logger(), "StateMachine entering EGM mode...");

    // Sending signal EGM_START_JOINT, triggering the interrupt.
    this->services().egm().signalEGMStartJoint(); 
    if( !((this->services().main().getCurrentState(task_L_) == 3) && (this->services().main().getCurrentState(task_R_) == 3 )))
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to start EGM mode.");
      return false;
    }

    // Giving the StateMachine time to return to main() and then enter runEGMRoutine()
    usleep(1*100000);
    RCLCPP_INFO(node_->get_logger(), "StateMachine in EGM mode");

    // To solve known issue with abb_libegm:
    // "RAPID execution state is UNDEFINED (might happen first time after controller start/restart). 
    // "Try to restart the RAPID program."
    if( ((this->services().main().getCurrentState(task_L_) == 4) || (this->services().main().getCurrentState(task_R_) == 4 )) )
    {
      RCLCPP_INFO(node_->get_logger(), "Known issue, 'RAPID execution state is UNDEFINED' encountered. Restarting StateMachine");
      this->stopRAPIDExecution();
      usleep(0.5*1000000);
      this->resetRAPIDProgramPointer();
      usleep(0.5*1000000);
      start_state_machine();
      usleep(0.5*1000000);
      go_to_state(requested_state_);
      RCLCPP_INFO(node_->get_logger(), "Issue solved."); // Not really
    }

    return true;
  }

  // STATE_RUN_RAPID_ROUTINE
  if(requested_state_ == "rapid")
  {
    RCLCPP_ERROR(node_->get_logger(), "Rapid mode not yet implemented");
    return false;
  }
  // Invalid input
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Could not evaluate requested state");
    return false;
  }
  
  requested_state_ = {};
  return false;
}


bool YumiRobotManager::configure()
{  
  if(calibrate_grippers())
  {
    is_ready_ = true;
    return true;
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Configuration failed");
    return false;
  }
}


void YumiRobotManager::spin()
{
  rclcpp::spin(node_);
}


//----------Helper Functions--------------------------------------------------------------------------------------------
bool YumiRobotManager::configure_egm()
{
  bool left_successfull = false;
  if(this->services().egm().getSettings(task_L_, egm_settings_l_.get())) // Fetchen funker ikke
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "**** setup_uc.use_filtering.value : " << egm_settings_l_->setup_uc.use_filtering.value);
    RCLCPP_INFO_STREAM(node_->get_logger(), "**** egm_settings_l_->setup_uc.comm_timeout.value : " << egm_settings_l_->setup_uc.comm_timeout.value );
    egm_settings_l_->setup_uc.use_filtering.value = false;
    egm_settings_l_->setup_uc.comm_timeout.value = 60.0;
    egm_settings_l_->run.ramp_in_time.value = 0.1;
    egm_settings_l_->run.pos_corr_gain.value  = 1.0;
    egm_settings_l_->activate.lp_filter.value = 0.0;
    egm_settings_l_->activate.sample_rate.value = 4.0;
    egm_settings_l_->activate.max_speed_deviation.value = 10.0;
    RCLCPP_INFO_STREAM(node_->get_logger(), "**** setup_uc.use_filtering.value : " << egm_settings_l_->setup_uc.use_filtering.value);
    RCLCPP_INFO_STREAM(node_->get_logger(), "**** egm_settings_l_->setup_uc.comm_timeout.value : " << egm_settings_l_->setup_uc.comm_timeout.value );
    if(this->services().egm().setSettings(task_L_, *egm_settings_l_.get()))
    {
      left_successfull = true;
      RCLCPP_INFO(node_->get_logger(), "writing to left StateMachine succesfull");
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "unable to write changes");
      return false;
    }
  }

  if(this->services().egm().getSettings(task_R_, egm_settings_r_.get()))
  {
    egm_settings_r_->setup_uc.use_filtering.value = false;
    egm_settings_r_->setup_uc.comm_timeout.value = 60.0;
    egm_settings_r_->run.ramp_in_time.value = 0.1;
    egm_settings_r_->run.pos_corr_gain.value  = 1.0;
    egm_settings_r_->activate.lp_filter.value = 0.0;
    egm_settings_r_->activate.sample_rate.value = 4.0;
    egm_settings_r_->activate.max_speed_deviation.value = 10.0;
    if(this->services().egm().setSettings(task_R_, *egm_settings_r_.get()))
    {
      RCLCPP_INFO(node_->get_logger(), "writing to right StateMachine succesfull");
      if(left_successfull) return true;
    }
  }
  return true;
  // RCLCPP_ERROR(node_->get_logger(), "Unable to adjust EGM settings of both StateMachines");
  // return false;
}


void YumiRobotManager::busy_wait_until_idle()
{
  while(!(this->services().main().isStateIdle(task_L_).isTrue() 
        && this->services().main().isStateIdle(task_R_).isTrue()))
  {
    usleep(0.1*1000000); 
  }
}


void YumiRobotManager::wait_for_gripper_to_finish_motion()
{
  // Assumption: Worst Case Execution Time < 2s
  usleep(2*1000000);
}


bool YumiRobotManager::calibrate_grippers()
{
  // RAPID execution check. Check if StateMachine is running.
  if(!this->isRAPIDRunning().isTrue())
  {
    RCLCPP_ERROR(node_->get_logger(), "Unable to calibrate SmartGrippers without StateMachine executing.");
    return false;
  }

  // While initializing the submodules the STATE is STATE_INITIALIZE. When compeleted it goes to STATE_IDLE.
  // If StateMachien is not ready yet, busy wait until STATE is STATE_IDLE.
  busy_wait_until_idle();

  // Calibrate Grippers. It is assumed grippers are closed.
  RCLCPP_INFO(node_->get_logger(), "Calibrating SmartGrippers...");
  if(!this->services().sg().dualCalibrate())
  {
    RCLCPP_ERROR(node_->get_logger(), "Could not Calibrate SmartGrippers");  
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "SmartGrippers calibrated..");
  
  // Wait until gripper is done calibrating. Assumption: Worst Case Execution Time < 3s
  usleep(2*1000000);
  busy_wait_until_idle();
  return true;
}


bool YumiRobotManager::stop_egm()
{
  if(!this->services().egm().signalEGMStop())
  {
    RCLCPP_WARN(node_->get_logger(), "Unable to stop EGM execution");
    return false;
  }
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
  response->is_ready = is_ready_;
}

void YumiRobotManager::handle_StopMotors(const std::shared_ptr<rmw_request_id_t> request_header,
                                    const std::shared_ptr<StopMotors::Request> request,
                                    const std::shared_ptr<StopMotors::Response> response)
{
  (void) request_header;
  (void) request;
  if(this->setMotorsOff())
  {
    if(this->isMotorOn().isFalse())
    {
      response->motors_off = true;
    }
    else
    {
      // If unable to verify motors are off, return false.
      response->motors_off = false;
    }
  }
  else
  {
    response->motors_off = false;
  }
}


} // namespace yumi_robot_manager