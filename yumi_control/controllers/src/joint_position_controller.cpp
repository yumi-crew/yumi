#include "controllers/joint_position_controller.hpp"

#include <string>
#include <sstream>
#include <random>
#include <memory>
#include <exception>

#include <angles/angles.h>

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rcutils/logging_macros.h"



namespace ros_controllers
{


// namespaced helper functions..........................................................................................
unsigned int random_char()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, 255);
  return dis(gen);
}

std::string generate_hex(const unsigned int len)
{
  std::stringstream ss;
  for (size_t i = 0; i < len; i++)
  {
    const auto rc = random_char();
    std::stringstream hexstream;
    hexstream << std::hex << rc;
    auto hex = hexstream.str();
    ss << (hex.length() < 2 ? '0' + hex : hex);
  }
  return ss.str();
}
// .....................................................................................................................


JointPositionController::JointPositionController()
    : controller_interface::ControllerInterface()
{
}


controller_interface::controller_interface_ret_t
JointPositionController::init(std::weak_ptr<hardware_interface::RobotHardware> robot_hardware,
                              const std::string &controller_name)
{
  // initialize lifecycle node
  auto ret = ControllerInterface::init(robot_hardware, controller_name);
  if (ret != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS)
  {
      RCLCPP_WARN(this->get_lifecycle_node()->get_logger(), "JointPositionController init() unsuccesful");
      return ret;
  }

  RCLCPP_INFO(this->get_lifecycle_node()->get_logger(), "JointPositionController init() successful");
  return controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS;
}


controller_interface::controller_interface_ret_t
JointPositionController::update()
{ 
  //  * State:    Active
  //  * Performs: Calculates error between desired and current pos and feedbacks it through the PID,
  //              then updates the "correct" cmd_handle from registered_joint_cmd_handles_         

  //            +-----+
  //    error-->|  P  |-->joint_addition
  //            +-----+

  auto timeNow = this->get_lifecycle_node()->get_clock()->now();
  auto timeElapsed = timeNow - previous_update_time_;
  previous_update_time_ = timeNow;
  auto pos_map = std::map<std::string, double>();

  // populating joint_name + joint_pos map
  for (auto &state : registered_joint_state_handles_)
  {
    pos_map[state->get_name()] = state->get_position();
  }

  // for every joint:
  for (size_t i = 0; i < registered_joint_state_handles_.size(); i++)
  {
    auto state_handle = registered_joint_state_handles_[i];
    auto joint_name = state_handle->get_name();
    auto curr_pos = state_handle->get_position();


    // find_if and utilised lambda function
    auto fp = [&joint_name](const hardware_interface::JointCommandHandle *cmd_handle) -> bool 
    { 
      // return true if its the same joint.
      return (cmd_handle->get_name().compare(joint_name) == 0); 
    };
    auto joint_cmd_iter = 
      std::find_if(registered_joint_cmd_handles_.cbegin(), registered_joint_cmd_handles_.cend(), fp);
    


    // if joint is to be controlled by this controller
    if (joint_cmd_iter != registered_joint_cmd_handles_.cend()) 
    {
      auto cmd_handle = *joint_cmd_iter;
      auto desired_pos = desired_pos_map_[joint_name];  

      if(curr_pos != desired_pos)
      {
        //--------------PID---------------------------------------------------------------------------------------------
        auto error = desired_pos - curr_pos;
        auto addition = pid_controllers_map_[joint_name]->compute_command(error, timeElapsed);
        // TODO : Replace with proper NaN handling
        if(std::isnan(addition))
        {
          addition = 0.0;
        }
        if(std::isnan(curr_pos))
        {
          curr_pos = 0.0;
        }
        //--------------------------------------------------------------------------------------------------------------
        //RCLCPP_INFO_STREAM(this->get_lifecycle_node()->get_logger(), "addition: " << addition);
        cmd_handle->set_cmd(curr_pos + addition);
        //cmd_handle->set_cmd(1);
      }   
    }
  }

  return hardware_interface::HW_RET_OK;
}

//########################## Callbacks #################################################################################

void 
JointPositionController::desired_position_subscrition_callback(ros2_control_interfaces::msg::JointControl::UniquePtr msg)
{
  //  * State:            Active
  //  * Performs:         Receives incoming msg, populates desired_pos_map_ 
  //  * if unsuccessfull: Prints warning logger message with info about the issue.

  // TODO: Possible optimisation: put this callback in RobotHW instead because most of the data here is not used anyway
  // However that optimisation will be ugly due to the need to modify the base RobotHW class as there is currently no 
  // handle for desired pos method


  auto msg_size = msg->goals.size();                                  
  auto names_size = msg->joints.size();                               
  auto cmd_handle_size = registered_joint_cmd_handles_.size();
  
  // TODO: Error handling when data don't match expectations
  if (names_size != msg_size)
  {
    names_size = 0;
    RCLCPP_WARN_ONCE(this->get_lifecycle_node()->get_logger(),
      "Number of joint names do not correspond to number of desired positions. Ignoring data");
    return;
  }

  if (msg_size > cmd_handle_size)
  {
    RCLCPP_WARN_ONCE(this->get_lifecycle_node()->get_logger(), 
      "Subscribed desired position more than robot can handle, adding to map anyway");
  }
  else if (msg_size < cmd_handle_size)
  {
    RCLCPP_WARN_ONCE(this->get_lifecycle_node()->get_logger(),
      "Subscribed desired position less than total joints in robot, ignoring control for joints at the end of the list");
  }

  // Add data to hash table
  for (size_t i = 0; i < msg_size; i++)
  {
    auto name = msg->joints[i];
    auto desired_pos = msg->goals[i];
    desired_pos_map_[name] = desired_pos;
  }

}


//########################## Node Lifecycle ############################################################################

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointPositionController::on_configure(const rclcpp_lifecycle::State &previous_state)
{
  //  * State:           Configuring
  //  * Performs:        Fetches joint_state and joint_cmd handles.
  //  * If FAILURE:      Node transitions to Unconfigured.
  //  * If SUCCESS:      Node transitions to Inactive.
  //  * If error raised: Node transitions to ErrorProcessing.

  (void)previous_state;
  namespace_ =  this->get_lifecycle_node()->get_parameter("namespace").as_string();   //namespacing
  auto controller_joints = get_controller_joints();   

  if (auto sptr = robot_hardware_.lock()) 
  {
    auto state_handles = sptr->get_registered_joint_state_handles();
    auto cmd_handles   = sptr->get_registered_joint_command_handles();
    
    // Register the appropriate handles based on what joints this controller is supposed to control
    for (auto &joint_name : controller_joints)
    {   
      auto fp = [&joint_name](const hardware_interface::JointStateHandle *state_handle) -> bool
      { 
        return state_handle->get_name().compare(joint_name) == 0; 
      };

      // TODO: change name of stateElem
      auto stateElem = std::find_if(state_handles.cbegin(), state_handles.cend(), fp);
      if (stateElem != state_handles.cend())
      {
        registered_joint_state_handles_.push_back(*stateElem);
      }

      auto fp2 = [&joint_name](const hardware_interface::JointCommandHandle *cmd_handle) -> bool 
      { 
        return (cmd_handle->get_name().compare(joint_name) == 0); 
      };

      auto cmdElem = std::find_if(cmd_handles.cbegin(), cmd_handles.cend(), fp2);
      if (cmdElem != cmd_handles.cend())
      {
        registered_joint_cmd_handles_.push_back(*cmdElem);
      }
    }

    //-----Error handling-----------------------------------------------------------------------------------------------
    if (registered_joint_cmd_handles_.size() != registered_joint_state_handles_.size())
    {
      RCLCPP_ERROR(this->get_lifecycle_node()->get_logger(), 
        "Number of state and command handles must be the same. Exiting.");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    
    if (registered_joint_state_handles_.size() == 0)
    {
      RCLCPP_ERROR(this->get_lifecycle_node()->get_logger(), "0 joint state handles registered. Exiting."); 
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    
    if (registered_joint_cmd_handles_.size() == 0)
    {
      RCLCPP_ERROR(this->get_lifecycle_node()->get_logger(), "0 joint command handles registered. Exiting.");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    //------------------------------------------------------------------------------------------------------------------

  } 
  else
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  previous_update_time_ = this->get_lifecycle_node()->get_clock()->now();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointPositionController::on_activate(const rclcpp_lifecycle::State &previous_state)
{
  //  * State:              Activating
  //  * Purpose:            Sets up the subscriber and loads PID controllers. 
  //  * If Error raised:    Return to Inactive state.
  
  (void) previous_state;
  RCLCPP_INFO(this->get_lifecycle_node()->get_logger(), "JointPositionController on_activate called");
  std::string robotName;

  if (auto robot_hardware = robot_hardware_.lock()) 
  {
  }
  else
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  subscription_ = this->get_lifecycle_node()->create_subscription<ros2_control_interfaces::msg::JointControl>(
    namespace_+"/joint_commands", rclcpp::SensorDataQoS(), 
    std::bind(&JointPositionController::desired_position_subscrition_callback, this, std::placeholders::_1)
    );

  auto pidParams = control_utils::Pid::Gains();
  pidParams = get_controller_pid();

  // Use negative p gain as a sign of error
  if (pidParams.p_gain_ < 0)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(this->get_lifecycle_node()->get_logger(), "Creating pid controllers");
  for (auto &j : registered_joint_cmd_handles_)
  {
    pid_controllers_map_[j->get_name()] = std::make_shared<control_utils::Pid>(pidParams);
  }

  // desired_pos = initial_pos until told otherwise
  int index = 0;
  for (auto &cmd_handle : registered_joint_cmd_handles_)
  {
    desired_pos_map_.insert({cmd_handle->get_name(), registered_joint_state_handles_[index]->get_position()});
    ++index;
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointPositionController::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
  //  * State:              Deactivating.
  //  * Purpose:            Cleanup/Reversing of the on_activate changes.
  //  * If Error raised:    Node transitions to ErrorProcessing.
  //  * If succesfull:      Node transitions to Inactive

  (void)previous_state;
  RCLCPP_INFO(this->get_lifecycle_node()->get_logger(), "JointPositionController on_deactivate called");
  pid_controllers_map_.clear();
  subscription_ = nullptr;
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
} 


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointPositionController::on_cleanup(const rclcpp_lifecycle::State &previous_state)
{
  //  * State:            CleaningUp
  //  * Purpose:          Clear all state and return the node to a equivalent state as when first created.
  //  * If Error raised:  Node transitions to ErrorProcessing
  //  * If succesfull:    Node transitions to Unconfigured

  (void)previous_state;

  registered_joint_state_handles_.clear();
  registered_joint_cmd_handles_.clear();
  desired_pos_map_.clear();

  RCLCPP_INFO(this->get_lifecycle_node()->get_logger(), "JointPositionController on_cleanup called");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
} 


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointPositionController::on_error(const rclcpp_lifecycle::State &previous_state)
{

  //  * State:            ErrorProcessing
  //  * Purpose:          Does: Info logger message. Should: Do Error handling.
  //  * If Error raised:  Does: - . Should: Node transition to finalized for destruction.
  //  * If succesfull:    Node transition to unconfigured.

  (void)previous_state;
  RCLCPP_INFO(this->get_lifecycle_node()->get_logger(), "JointPositionController on_error called");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
} 


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointPositionController::on_shutdown(const rclcpp_lifecycle::State &previous_state)
{
  //  * State:             ShuttingDown
  //  * Purpose:           Perform cleanup necessary before destruction. 
  //  * Ìf Error raised:   Does: - . Should: Node transition to ErrorProcessing.
  //  * If succesfull:     Does: Node transition to finalized.

  (void)previous_state;
  RCLCPP_INFO(this->get_lifecycle_node()->get_logger(), "JointPositionController on_shutdown called");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
} 


//########################## Class helper functions ####################################################################



//helper function
control_utils::Pid::Gains   //Kp, Ki, Kd
JointPositionController::get_controller_pid()
{
  
  using GetControllerPid = parameter_server_interfaces::srv::GetControllerPid;
  using namespace std::chrono_literals;

  auto gain = control_utils::Pid::Gains();
  auto temp_node = std::make_unique<rclcpp::Node>("temp_"+generate_hex(8));
  auto client = temp_node->create_client<GetControllerPid>(namespace_+"/GetControllerPid"); 

  // loop logic
  unsigned int retryCount = 0;
  constexpr unsigned int maxRetries = 10;
  while (retryCount < maxRetries) 
  {
    client->wait_for_service(1.5s);
    if (!client->service_is_ready())
    {
      retryCount++;  
      RCLCPP_ERROR(this->get_lifecycle_node()->get_logger(),
        "GetControllerPid service failed to start, check that parameter server is launched. Retries left: %d", 
        maxRetries - retryCount);
      continue;
    }

    auto req = std::make_shared<parameter_server_interfaces::srv::GetControllerPid::Request>();
    req->controller = this->get_lifecycle_node()->get_name();
    auto resp = client->async_send_request(req);
    RCLCPP_INFO(this->get_lifecycle_node()->get_logger(), "Getting PID parameters for controller %s...", req->controller.c_str());
    
    auto spin_status = rclcpp::spin_until_future_complete(temp_node->get_node_base_interface(), resp, 3s);
    if (spin_status != rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      retryCount++;
      RCLCPP_ERROR(this->get_lifecycle_node()->get_logger(),
        "GetControllerPid service failed to execute (spin failed). Retries left: %d", maxRetries - retryCount);
      continue;
    }

    auto status = resp.wait_for(1s);
    if (status != std::future_status::ready)
    {
      retryCount++;
      RCLCPP_ERROR(this->get_lifecycle_node()->get_logger(), 
        "GetControllerPid service failed to execute. Retries left: %d", maxRetries - retryCount);
      continue;
    }

    auto res = resp.get();
    gain.p_gain_ = res->p;
    gain.i_gain_ = res->i;
    gain.d_gain_ = res->d;
    gain.i_max_ = res->i_max;
    gain.i_min_ = res->i_min;
    gain.antiwindup_ = res->antiwindup;
    return gain;
  }

  RCLCPP_FATAL(this->get_lifecycle_node()->get_logger(), 
    "Get controller gain failed for controller %s", this->get_lifecycle_node()->get_name());
  gain.p_gain_ = -1;
  return gain;
}


// helper function
std::vector<std::string> 
JointPositionController::get_controller_joints()
{

  using GetControllerJoints = parameter_server_interfaces::srv::GetControllerJoints;
  using namespace std::chrono_literals;

  std::vector<std::string> controller_joints = {};
  auto temp_node = std::make_unique<rclcpp::Node>("temp_"+generate_hex(8));
  auto client = temp_node->create_client<GetControllerJoints>(namespace_+"/GetControllerJoints");  

  // loop logic
  unsigned int retryCount = 0;
  constexpr unsigned int maxRetries = 10;
  while (retryCount < maxRetries)
  {
    client->wait_for_service(1.5s);
    if (!client->service_is_ready()) 
    {
      retryCount++;
      RCLCPP_ERROR(this->get_lifecycle_node()->get_logger(),
        "GetControllerJoints service failed to start, check that parameter server is launched. Retries left: %d", 
        maxRetries - retryCount);
      continue;
    }

    auto req = std::make_shared<parameter_server_interfaces::srv::GetControllerJoints::Request>();
    req->controller = this->get_lifecycle_node()->get_name();
    auto resp = client->async_send_request(req);
    RCLCPP_INFO(this->get_lifecycle_node()->get_logger(), "Getting joints for controller %s...", req->controller.c_str());
    auto spin_status = rclcpp::spin_until_future_complete(temp_node->get_node_base_interface(), resp, 3s);

    if (spin_status != rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      retryCount++;
      RCLCPP_ERROR(this->get_lifecycle_node()->get_logger(), 
        "GetControllerJoints service failed to execute (spin failed). Retries left: %d", maxRetries - retryCount);
      continue;
    }
    auto status = resp.wait_for(1s);

    if (status != std::future_status::ready)
    {
      retryCount++;
      RCLCPP_ERROR(this->get_lifecycle_node()->get_logger(), 
        "GetControllerJoints service failed to execute. Retries left: %d", maxRetries - retryCount);
      continue;
    }

    auto res = resp.get();
    return res->joints;
  }

  RCLCPP_FATAL(this->get_lifecycle_node()->get_logger(), 
    "Unable to get joints for controller %s", this->get_lifecycle_node()->get_name());
  return {};
}




} // namespace ros_controllers



#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
    ros_controllers::JointPositionController, controller_interface::ControllerInterface) 