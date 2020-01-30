// Copyright 2019 Norwegian University of Science and Technology.
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

#include <abb_egm_hardware/abb_egm_hardware_sim.h>

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
{
}


hardware_interface::hardware_interface_ret_t 
AbbEgmHardware::init()
{
  node_ = rclcpp::Node::make_shared(name_);
  namespace_ = node_->get_namespace();
  auto ret = hardware_interface::HW_RET_ERROR;

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
  
  // fix
  if(n_joints_ == 7)
    configuration_.axes = abb::egm::RobotAxes::Seven;
  if(n_joints_ == 6)
    configuration_.axes = abb::egm::RobotAxes::Six;

 

  // then read in the states of the robot so the controllers can fetch it upon activation
  //this->read();


  return hardware_interface::HW_RET_OK;
}



hardware_interface::hardware_interface_ret_t 
AbbEgmHardware::read()
{
  return hardware_interface::HW_RET_OK;
}



hardware_interface::hardware_interface_ret_t 
AbbEgmHardware::write()
{ 
  usleep(1*100000); //0.1 seconds
	for (size_t index = 0; index < n_joints_; ++index)
	{
			joint_position_[index] = joint_position_command_[index];
	}
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
  // Set size of vectors so no dynamic memory allocation takes place in control loop.
  joint_state_handles_.resize(n_joints_);
  joint_command_handles_.resize(n_joints_);
  read_op_handles_.resize(n_joints_);
  write_op_handles_.resize(n_joints_);

  // Set size and zero intialize
  joint_position_.assign(n_joints_, 0.0);
  joint_velocity_.assign(n_joints_, 0.0);
  joint_effort_.assign(n_joints_, 0.0);
  joint_position_command_.assign(n_joints_, 0.0);

  for (int i = 0; i < n_joints_; ++i)
  {
    read_op_[i] = false;
    write_op_[i] = true;
  }

  
  return hardware_interface::HW_RET_OK;
}


}  // namespace abb_egm_hardware
