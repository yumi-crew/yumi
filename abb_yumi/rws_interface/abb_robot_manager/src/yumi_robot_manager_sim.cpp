#include <abb_robot_manager/yumi_robot_manager_sim.h>

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
name_(name)
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

  return true;
}



bool 
YumiRobotManager::go_to_state(std::string mode)
{
    return true;
}


bool 
YumiRobotManager::run_setup_tests()
{
  return true;
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
  return true;
}


void
YumiRobotManager::busy_wait_until_idle()
{
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
  return true;
}



bool
YumiRobotManager::stop_egm()
{
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
  response->is_ready = true;
}


} // namespace yumi_robot_manager