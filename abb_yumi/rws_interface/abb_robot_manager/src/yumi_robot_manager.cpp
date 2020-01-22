#include <abb_robot_manager/yumi_robot_manager.hpp>

/*  YUMI ABB RWS ROBOT MANAGER
 *  
 *  Responsibility: * Is the virtual replacement of the operator configuring the robot to be controlled in 
 *                    Externally Guided Motion (EGM) mode.
 *                   
 *  Functionalities: * Provides checking to confirm robot is connected, is in auto mode and have motors on.
 *                     Auto mode must be configured manually, other operations like turning motors on and off is
 *                     is handled using the Robot Web Services (RWS).
 * 
 *                   * Provides fetching and setting of settings in the robot system. Currently only adjusting of EGM 
 *                     and RWS settings are implemented in the manager.
 *      
 *                   * Starts the StateMachine on the robot controller.
 */



// TODO:  * Change most of the prints to only print if log_severity is set to DEBUG.

namespace abb_robot_manager
{


YumiRobotManager::YumiRobotManager(const std::string &name, const std::string &ip_address) 
: 
name_(name),
abb::rws::RWSStateMachineInterface(ip_address)
{
}


bool
YumiRobotManager::init()
{
  //--------------------------------------------------------------------------------------------------------------------
  // Startup. Performs checking of StateMachine's required conditions.
  //--------------------------------------------------------------------------------------------------------------------
  

  // Construction
  node_ = rclcpp::Node::make_shared(name_); 
  egm_settings_.reset(new abb::rws::RWSStateMachineInterface::EGMSettings);
  sg_settings_l_.reset(new abb::rws::RWSStateMachineInterface::SGSettings);
  sg_settings_r_.reset(new abb::rws::RWSStateMachineInterface::SGSettings);



  // Connection check. Checks if robot controller is connected. Loops until connection is made.
  auto runtime_info = this->collectRuntimeInfo();
  RCLCPP_INFO(node_->get_logger(), "Connecting to Robot...");
  while(!runtime_info.rws_connected)
  {
    RCLCPP_INFO(node_->get_logger(), "Connection failed. Check robot is connected. Retrying...");
    runtime_info = this->collectRuntimeInfo();
    usleep(2*1000000);
  }
  RCLCPP_INFO(node_->get_logger(), "Succesfully connected to Robot.");



  // Auto mode check. Check if robot controller is in Auto mode. 
  // unexpected movements.
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
    usleep(1*100000);
    if(this->isRAPIDRunning().isTrue())
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to stop StateMachine");
      return false;
    }
  }
  RCLCPP_INFO(node_->get_logger(), "StateMachine confirmed not running.");

  // Setting up service server

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

  return true;
}



bool
YumiRobotManager::start_state_machine()
{
  //--------------------------------------------------------------------------------------------------------------------
  // Start StateMachine 
  //--------------------------------------------------------------------------------------------------------------------
   
  // Motor check. Check if motors are on, turn on if not.
  RCLCPP_INFO(node_->get_logger(), "Checking if motors are on...");
  if(!this->isMotorOn().isTrue())
  {
    if(!this->setMotorsOn())
    {
      RCLCPP_INFO(node_->get_logger(), "Not able to turn on motors");
    }
  }
  RCLCPP_INFO(node_->get_logger(), "Motors are on.");  



  // Reset program pointer.
  if(!first_execution_)
  {
    RCLCPP_INFO(node_->get_logger(), "Reseting program pointer ...");       
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



bool 
YumiRobotManager::go_to_state(std::string mode)
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
      RCLCPP_INFO(node_->get_logger(), "Issue solved.");
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


bool 
YumiRobotManager::run_setup_tests()
{
  return test_grippers();
}


void 
YumiRobotManager::spin()
{
    rclcpp::spin(node_);
}

//----------Helper Functions--------------------------------------------------------------------------------------------

bool 
YumiRobotManager::get_configuration_data()
{
  // EGM settings
  egm_configuration_data_.setup_uc_settings.comm_timeout  = 30.0;
  
  egm_configuration_data_.activate_settings.tool.robhold          = true;
  egm_configuration_data_.activate_settings.cond_min_max          = 0.5;
  egm_configuration_data_.activate_settings.lp_filter             = 0.0;
  egm_configuration_data_.activate_settings.max_speed_deviation   = 400.0;

  egm_configuration_data_.run_settings.cond_time      = 10.0;
  egm_configuration_data_.run_settings.ramp_in_time   = 0.1;
  egm_configuration_data_.run_settings.pos_corr_gain  = 1.0;

  //SG settings
  sg_configuration_data_.max_speed   = 10;
  sg_configuration_data_.hold_force  = 5;

  return true;
}


void
YumiRobotManager::busy_wait_until_idle()
{
  while( !(this->services().main().isStateIdle(task_L_).isTrue() && this->services().main().isStateIdle(task_R_).isTrue()) )
  {
    usleep(0.1*1000000); 
  }
}


void
YumiRobotManager::wait_for_gripper_to_finish_motion()
{
  // Assumption: Worst Case Execution Time < 2s
  usleep(2*1000000);
}


bool 
YumiRobotManager::test_grippers()
{

  /* Calibrates and Tests both SmartGrippers, confirms they are moveable. Performs open then close operation. */


  // RAPID execution check. Check if StateMachine is running.
  if(!this->isRAPIDRunning().isTrue())
  {
    RCLCPP_ERROR(node_->get_logger(), "Unable to test SmartGrippers without StateMachine executing.");
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
    // return false;
  }
  RCLCPP_INFO(node_->get_logger(), "SmartGrippers calibrated..");
  

  // Wait until gripper is done calibrating. Assumption: Worst Case Execution Time < 3s
  usleep(2*1000000);
  

  // Test: open grippers.
  if( (this->services().main().isStateIdle(task_L_).isTrue() && this->services().main().isStateIdle(task_R_).isTrue()) )
  {
    // Perform open operation.
    RCLCPP_INFO(node_->get_logger(), "Testing, opening SmartGrippers...");
    if(!this->services().sg().dualGripOut())
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to send signal COMMAND_GRIP_OUT");
    }

    wait_for_gripper_to_finish_motion();
  }


  busy_wait_until_idle();


  // Test: close grippers.
  if( (this->services().main().isStateIdle(task_L_).isTrue() && this->services().main().isStateIdle(task_R_).isTrue()) )
  {
    // Perform close operation.
    RCLCPP_INFO(node_->get_logger(), "Testing, closing SmartGrippers...");
    if(!this->services().sg().dualGripIn())
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to send signal COMMAND_GRIP_IN");
    }

    wait_for_gripper_to_finish_motion();
  }


  is_ready_ = true;
  return true;
}



bool
YumiRobotManager::stop_egm()
{
  if(!this->services().egm().signalEGMStop())
  {
    RCLCPP_WARN(node_->get_logger(), "Unable to stop EGM execution");
    return false;
  }
  return true;
}



//---------- Service server handler functions---------------------------------------------------------------------------

void 
YumiRobotManager::handle_StopEgm(const std::shared_ptr<rmw_request_id_t> request_header,
                                    const std::shared_ptr<StopEgm::Request> request,
                                    const std::shared_ptr<StopEgm::Response> response)
{
 (void) request_header;
  if(request->to_stop)
  {
    if(stop_egm())
    {
      response->is_stopped = true;
    }
    else
    {
      response->is_stopped = false;
    }
  }
}


void
YumiRobotManager::handle_StartEgm(const std::shared_ptr<rmw_request_id_t> request_header,
                                     const std::shared_ptr<StartEgm::Request> request,
                                     const std::shared_ptr<StartEgm::Response> response)
{   
  (void) request_header;
  if(request->to_start)
  {
    if(go_to_state("egm"))
    {
      response->is_started = true;
    }
    else
    {
      response->is_started = false;
    } 
  }
}


void
YumiRobotManager::handle_IsReady(const std::shared_ptr<rmw_request_id_t> request_header,
                                    const std::shared_ptr<IsReady::Request> request,
                                    const std::shared_ptr<IsReady::Response> response)
{   
  (void) request_header;
  response->is_ready = is_ready_;
}


} // namespace yumi_robot_manager