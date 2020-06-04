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

#include <abb_egm_hardware/abb_egm_hardware.hpp>

namespace abb_egm_hardware
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


AbbEgmHardware::AbbEgmHardware(const std::string& name) : name_(name)
{}


hardware_interface::hardware_interface_ret_t 
AbbEgmHardware::init()
{
  node_ = rclcpp::Node::make_shared(name_);
  namespace_ = node_->get_namespace();
  auto ret = hardware_interface::HW_RET_ERROR;

    // RML
  n_joints_ = 7;
  rml_.reset(new ReflexxesAPI(n_joints_, 0.004));
  rml_input_.reset(new RMLPositionInputParameters(n_joints_));
  rml_output_.reset(new RMLPositionOutputParameters(n_joints_));

  ret = get_robot_name();
  if (ret != hardware_interface::HW_RET_OK)
  {
    RCLCPP_WARN(node_->get_logger(), "Initialization failed. Not able to fetch name of robot from the paramter server");
    return ret;
  } 

  ret = get_port();
  if (ret != hardware_interface::HW_RET_OK)
  {
    RCLCPP_WARN(node_->get_logger(), "Initialization failed. Not able to fetch port number from the parameter server");
    return ret;
  }

  ret = get_joint_names();
  if (ret != hardware_interface::HW_RET_OK)
  {
    RCLCPP_WARN(node_->get_logger(), "Initialization failed. Not able to fetch joint names from the parameter server");
    return ret;
  }

  // Define the length of the vectors so no dynamic memory allocation occurs during the control loop.
  initialize_vectors();

  // register all the handles for all the joints
  for (std::size_t i = 0; i < n_joints_; ++i)
  {
    joint_state_handles_[i] = hardware_interface::JointStateHandle(joint_names_[i], &joint_position_[i],
                                                                   &joint_velocity_[i], &joint_effort_[i]);

    ret = register_joint_state_handle(&joint_state_handles_[i]);
    if (ret != hardware_interface::HW_RET_OK)
    {
      RCLCPP_WARN(node_->get_logger(), "Can't register joint state handle %s", joint_names_[i].c_str());
      return ret;
    }

    joint_command_handles_[i] = hardware_interface::JointCommandHandle(joint_names_[i], &joint_position_command_[i]);
    ret = register_joint_command_handle(&joint_command_handles_[i]);
    if (ret != hardware_interface::HW_RET_OK)
    {
      RCLCPP_WARN(node_->get_logger(), "Can't register joint command handle %s", joint_names_[i].c_str());
      return ret;
    }



    joint_command_handles_vel_[i] = hardware_interface::JointCommandHandle(joint_names_[i] + "_vel", &joint_velocity_command_[i]);
    ret = register_joint_command_handle(&joint_command_handles_vel_[i]);
    if (ret != hardware_interface::HW_RET_OK)
    {
      RCLCPP_WARN(node_->get_logger(), "Can't register joint command handle %s", (joint_names_[i] + "_vel").c_str());
      return ret;
    }

    read_op_handles_[i] = hardware_interface::OperationModeHandle(
    read_op_handle_names_[i], reinterpret_cast<hardware_interface::OperationMode*>(&read_op_[i]));

    ret = register_operation_mode_handle(&read_op_handles_[i]);
    if (ret != hardware_interface::HW_RET_OK)
    {
      RCLCPP_WARN(node_->get_logger(), "Can't register read operation mode handle %s",
                  read_op_handle_names_[i].c_str());
      return ret;
    }

    write_op_handles_[i] = hardware_interface::OperationModeHandle(
        write_op_handle_names_[i], reinterpret_cast<hardware_interface::OperationMode*>(&write_op_[i]));

    ret = register_operation_mode_handle(&write_op_handles_[i]);
    if (ret != hardware_interface::HW_RET_OK)
    {
      RCLCPP_WARN(node_->get_logger(), "can't register write operation mode handle %s",
                  write_op_handle_names_[i].c_str());
      return ret;
    }
  }
  
  // Create an EGM interface:
  // * Sets up an EGM server (that the robot controller's EGM client can connect to).
  // * Provides APIs to the user (for setting motion references, that are sent in reply to the EGM client's request).
  configuration_.axes = num_axes_;
  configuration_.use_velocity_outputs = true; // Must be set for velocity control
  egm_interface_.reset(new abb::egm::EGMControllerInterface(io_service_, port_, configuration_));

  if (!egm_interface_->isInitialized())
  {
    RCLCPP_ERROR(node_->get_logger(), "EGM interface failed to initialize (e.g. due to port already bound)");
    return hardware_interface::HW_RET_ERROR;
  }

  // Spin up a thread to run the io_service.
  thread_group_.create_thread(boost::bind(&boost::asio::io_service::run, &io_service_));

  RCLCPP_WARN(node_->get_logger(), "Wait for an EGM communication session to start...");
  bool wait = true;
  while (rclcpp::ok() and wait)
  {
    RCLCPP_INFO(node_->get_logger(), "Wait for an EGM communication session to start...");
    if (egm_interface_->isConnected())
    {
      if (egm_interface_->getStatus().rapid_execution_state() ==
          abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_UNDEFINED)
      {
        RCLCPP_ERROR(node_->get_logger(), "RAPID execution state is UNDEFINED (might happen first time after "
                                          "controller start/restart). Try to restart the RAPID program.");
      }
      else
      {
        wait = egm_interface_->getStatus().rapid_execution_state() !=
               abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_RUNNING;
      }
    }
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  // Ensure command_ is empty
  command_.Clear();
  return hardware_interface::HW_RET_OK;
}


hardware_interface::hardware_interface_ret_t 
AbbEgmHardware::read()
{
  // Wait for a new EGM message from the EGM client (with a timeout of 500 ms).
  if (egm_interface_->waitForMessage(500))
  {
    // read recieved message into class variable state_
    egm_interface_->read(&state_);
    sequence_number_ = state_.header().sequence_number();

    if (first_packet_)
    {
      first_packet_ = false;
      abb::egm::wrapper::Joints initial_position;
      abb::egm::wrapper::Joints initial_velocity; 
      initial_position.CopyFrom(state_.feedback().robot().joints().position());
      initial_velocity.CopyFrom(state_.feedback().robot().joints().velocity());

      for (size_t index = 0; index < n_joints_; ++index)
      {
        joint_position_command_[index] = angles::from_degrees(initial_position.values(index));
        joint_velocity_command_[index] = angles::from_degrees(initial_velocity.values(index));

        /* For rml interpolation of segments */
        rml_input_->CurrentPositionVector->VecData[index] = joint_position_command_[index];
        rml_input_->CurrentVelocityVector->VecData[index] = joint_velocity_command_[index];
        rml_input_->CurrentAccelerationVector->VecData[index] = 0.0;
        rml_input_->SelectionVector->VecData[index] = true;

        rml_input_->MaxVelocityVector->VecData[index] = 7.0;
        rml_input_->MaxAccelerationVector->VecData[index] = 100.0;
        rml_input_->MaxJerkVector->VecData[index] = 1000.0;
      }

      // clear command_ to be sure it is empty
      command_.Clear();
      command_.mutable_robot()->mutable_joints()->mutable_position()->CopyFrom(initial_position); 
      command_.mutable_robot()->mutable_joints()->mutable_velocity()->CopyFrom(initial_velocity);  

      // rml_command_.Clear();
      // rml_command_.mutable_robot()->mutable_joints()->mutable_velocity()->CopyFrom(initial_velocity);     
    }
   
    for (size_t i = 0; i < n_joints_; ++i)
    {
      joint_position_[i] = angles::from_degrees(state_.feedback().robot().joints().position().values(i));
      joint_velocity_[i] = angles::from_degrees(state_.feedback().robot().joints().velocity().values(i));
    }
  }

  return hardware_interface::HW_RET_OK;
}


hardware_interface::hardware_interface_ret_t 
AbbEgmHardware::write()
{ 
  /* for rml interpolation of segments */
  for (std::size_t i = 0; i < n_joints_; ++i)
  {
    rml_input_->CurrentPositionVector->VecData[i] = joint_position_[i];
    rml_input_->CurrentVelocityVector->VecData[i] = joint_velocity_[i];

    rml_input_->TargetPositionVector->VecData[i] = joint_position_command_[i];
    rml_input_->TargetVelocityVector->VecData[i] = joint_velocity_command_[i];

  }
  int result = rml_->RMLPosition(*rml_input_.get(), rml_output_.get(), rml_flags_);
  if (result < 0)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "rml error: " << result);
  }


  // writes joint_position_command_ to command_ which is written to robot
  for (size_t index = 0; index < n_joints_; ++index)
  {

    rml_input_->CurrentAccelerationVector->VecData[index] = rml_output_->NewAccelerationVector->VecData[index];

    // std::cout << joint_velocity_command_[index] << " " << rml_output_->NewVelocityVector->VecData[index] << std::endl;

    // joint_position_command_[index] = rml_output_->NewPositionVector->VecData[index];
    // joint_velocity_command_[index] = rml_output_->NewVelocityVector->VecData[index];

    // rml_command_.mutable_robot()->mutable_joints()->mutable_position()->set_values(index, 
    //     angles::to_degrees(rml_output_->NewPositionVector->VecData[index]));
    // rml_command_.mutable_robot()->mutable_joints()->mutable_velocity()->set_values(index, 
    //     angles::to_degrees(rml_output_->NewVelocityVector->VecData[index]));


    command_.mutable_robot()->mutable_joints()->mutable_position()->set_values(index, 
        angles::to_degrees(joint_position_command_[index]));
    command_.mutable_robot()->mutable_joints()->mutable_velocity()->set_values(index, 
     angles::to_degrees(joint_velocity_command_[index]));
  }

  // std::cout << "\n";

  //command_.PrintDebugString();
  //rml_command_.PrintDebugString();
  
  // egm_interface_->write(rml_command_);
  egm_interface_->write(command_);
  return hardware_interface::HW_RET_OK;
}


hardware_interface::hardware_interface_ret_t
AbbEgmHardware::get_port()
{
  using GetPort = parameter_server_interfaces::srv::GetPort;
  using namespace std::chrono_literals;

  // temp node which will handle communication with the parameter server
  auto temp_node = std::make_unique<rclcpp::Node>("temp_"+generate_hex(8));
  auto client = temp_node->create_client<GetPort>(namespace_+"/GetPort");               

  // loop logic
  unsigned int retryCount = 0;
  constexpr unsigned int maxRetries = 10;

  while (retryCount < maxRetries)
  {
    client->wait_for_service(1.5s);
    if (!client->service_is_ready()) 
    {
      retryCount++;
      RCLCPP_ERROR(node_->get_logger(),
        "GetPort service failed to start, check that parameter server is launched. Retries left: %d", 
        maxRetries - retryCount);
      continue;
    }

    auto req = std::make_shared<parameter_server_interfaces::srv::GetPort::Request>();
    req->robot = robot_name_;
    auto resp = client->async_send_request(req);
    RCLCPP_INFO(node_->get_logger(), "Fetching communication port from parameter server");
    
    auto spin_status = rclcpp::spin_until_future_complete(temp_node->get_node_base_interface(), resp, 3s);
    if (spin_status != rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      retryCount++;
      RCLCPP_ERROR(node_->get_logger(), 
        "Getport service failed to execute (spin failed). Retries left: %d", maxRetries - retryCount);
      continue;
    }

    auto status = resp.wait_for(1s);
    if (status != std::future_status::ready)
    {
      retryCount++;
      RCLCPP_ERROR(node_->get_logger(), 
        "GetPort service failed to execute. Retries left: %d", maxRetries - retryCount);
      continue;
    }

    port_ = resp.get()->port;
    return hardware_interface::HW_RET_OK;
  }

  RCLCPP_ERROR(node_->get_logger(), "EGM interface failed to fetch port from parameter_server");
  return hardware_interface::HW_RET_ERROR;
}


hardware_interface::hardware_interface_ret_t
AbbEgmHardware::get_joint_names()
{
  using GetAllJoints = parameter_server_interfaces::srv::GetAllJoints;
  using namespace std::chrono_literals;

  // temp node which will handle communication with the parameter server
  auto temp_node = std::make_unique<rclcpp::Node>("temp_"+generate_hex(8));
  auto client = temp_node->create_client<GetAllJoints>(namespace_+"/GetAllJoints");    

  // loop logic
  unsigned int retryCount = 0;
  constexpr unsigned int maxRetries = 10;

  while (retryCount < maxRetries)
  {
    client->wait_for_service(1.5s);
    if (!client->service_is_ready()) 
    {
      retryCount++;
      RCLCPP_ERROR(node_->get_logger(),
        "GetAllJoints service failed to start, check that parameter server is launched. Retries left: %d", 
        maxRetries - retryCount);
      continue;
    }

    auto req = std::make_shared<parameter_server_interfaces::srv::GetAllJoints::Request>();
    req->robot = robot_name_;
    auto resp = client->async_send_request(req);
    RCLCPP_INFO(node_->get_logger(), "Fetching joint names from parameter server");
    
    auto spin_status = rclcpp::spin_until_future_complete(temp_node->get_node_base_interface(), resp, 3s);
    if (spin_status != rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      retryCount++;
      RCLCPP_ERROR(node_->get_logger(), 
        "GetAllJoints service failed to execute (spin failed). Retries left: %d", maxRetries - retryCount);
      continue;
    }

    auto status = resp.wait_for(1s);
    if (status != std::future_status::ready)
    {
      retryCount++;
      RCLCPP_ERROR(node_->get_logger(), 
        "GetAllJoints service failed to execute. Retries left: %d", maxRetries - retryCount);
      continue;
    }

    joint_names_ = resp.get()->joints;
    n_joints_ = joint_names_.size();
    return hardware_interface::HW_RET_OK;
  }

  RCLCPP_ERROR(node_->get_logger(), "EGM interface failed to fetch joint names from parameter_server");
  return hardware_interface::HW_RET_ERROR;
}


hardware_interface::hardware_interface_ret_t 
AbbEgmHardware::get_robot_name()
{
  using GetRobot = parameter_server_interfaces::srv::GetRobot;
  using namespace std::chrono_literals;

  // temp node which will handle communication with the parameter server
  auto temp_node = std::make_unique<rclcpp::Node>("temp_"+generate_hex(8));
  auto client = temp_node->create_client<GetRobot>(namespace_+"/GetRobot");           
  
  // loop logic
  unsigned int retryCount = 0;
  constexpr unsigned int maxRetries = 10;

  while (retryCount < maxRetries)
  {
    client->wait_for_service(1.5s);
    if (!client->service_is_ready()) 
    {
      retryCount++;
      RCLCPP_ERROR(node_->get_logger(),
        "GetRobot service failed to start, check that parameter server is launched. Retries left: %d", 
        maxRetries - retryCount);
      continue;
    }

    auto req = std::make_shared<parameter_server_interfaces::srv::GetRobot::Request>();
    auto resp = client->async_send_request(req);
    RCLCPP_INFO(node_->get_logger(), "Fetching robot name from parameter server");
    
    auto spin_status = rclcpp::spin_until_future_complete(temp_node->get_node_base_interface(), resp, 3s);
    if (spin_status != rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      retryCount++;
      RCLCPP_ERROR(node_->get_logger(), 
        "GetRobot service failed to execute (spin failed). Retries left: %d", maxRetries - retryCount);
      continue;
    }

    auto status = resp.wait_for(1s);
    if (status != std::future_status::ready)
    {
      retryCount++;
      RCLCPP_ERROR(node_->get_logger(), 
        "GetRobot service failed to execute. Retries left: %d", maxRetries - retryCount);
      continue;
    }

    robot_name_ =  resp.get()->robot;
    return hardware_interface::HW_RET_OK;
  }

  RCLCPP_ERROR(node_->get_logger(), "EGM interface failed to fetch joint names from parameter_server");
  return hardware_interface::HW_RET_ERROR;
}


hardware_interface::hardware_interface_ret_t 
AbbEgmHardware::initialize_vectors()
{
  // Set sizes
  joint_state_handles_.resize(n_joints_);
  joint_command_handles_.resize(n_joints_);
  joint_command_handles_vel_.resize(n_joints_);
  read_op_handles_.resize(n_joints_);
  write_op_handles_.resize(n_joints_);
  read_op_ = new bool[n_joints_];
  write_op_ = new bool[n_joints_];

  // Set sizes and zero intialize
  joint_position_.assign(n_joints_, 0.0);
  joint_velocity_.assign(n_joints_, 0.0);
  joint_effort_.assign(n_joints_, 0.0);
  joint_position_command_.assign(n_joints_, 0.0);
  joint_velocity_command_.assign(n_joints_, 0.0);
  
  for (int i = 0; i < n_joints_; ++i)
  {
    read_op_[i] = false;
    write_op_[i] = true;
  }

  return hardware_interface::HW_RET_OK;
}

}  // namespace abb_egm_hardware
