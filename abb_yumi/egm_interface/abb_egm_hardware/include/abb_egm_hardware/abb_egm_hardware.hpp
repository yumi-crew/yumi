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

#ifndef ABB_EGM_HARDWARE__ABB_EGM_HARDWARE_HPP_
#define ABB_EGM_HARDWARE__ABB_EGM_HARDWARE_HPP_

#include <memory>
#include <string>
#include <sstream>
#include <random>

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>

#include <angles/angles.h>

#include <hardware_interface/robot_hardware.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include <abb_libegm/egm_controller_interface.h>
#include <abb_libegm/egm_wrapper.pb.h>

#include <abb_egm_hardware/visibility_control.h>

// parameter server services
#include "parameter_server_interfaces/srv/get_port.hpp"
#include "parameter_server_interfaces/srv/get_all_joints.hpp"
#include "parameter_server_interfaces/srv/get_robot.hpp"




namespace abb_egm_hardware
{
class AbbEgmHardware : public hardware_interface::RobotHardware
{
public:
  AbbEgmHardware(const std::string& name);

  ABB_EGM_HARDWARE_PUBLIC
  hardware_interface::hardware_interface_ret_t init();

  ABB_EGM_HARDWARE_PUBLIC
  hardware_interface::hardware_interface_ret_t read();

  ABB_EGM_HARDWARE_PUBLIC
  hardware_interface::hardware_interface_ret_t write();


private:

  std::string name_;
  std::string robot_name_;
  std::shared_ptr<rclcpp::Node> node_;
  std::string namespace_;

  // Handles
  std::vector<hardware_interface::JointStateHandle> joint_state_handles_;
  std::vector<hardware_interface::JointCommandHandle> joint_command_handles_;
  std::vector<hardware_interface::OperationModeHandle> read_op_handles_;
  std::vector<hardware_interface::OperationModeHandle> write_op_handles_;

  // Boost components for managing asynchronous UDP socket(s).
  boost::asio::io_service io_service_;
  boost::thread_group thread_group_;

  // Udp endpoint robot will accept commands from.
  unsigned short port_; 

  abb::egm::BaseConfiguration configuration_;
  std::unique_ptr<abb::egm::EGMControllerInterface> egm_interface_;
  abb::egm::wrapper::Input state_;
  abb::egm::wrapper::Output command_;

  unsigned int sequence_number_ = 0.0;
  bool first_packet_ = true;

  unsigned short n_joints_;
  std::vector<std::string> joint_names_; 
  
  std::vector<double> joint_position_; 
  std::vector<double> joint_velocity_; 
  std::vector<double> joint_effort_; 
  std::vector<double> joint_position_command_; 

  // maximum number of joints of 10 implied here
  std::array<bool, 10> read_op_; 
  std::array<bool, 10> write_op_; 

  // fix, should be read from config file
  std::array<std::string, 7> read_op_handle_names_ = { "read1", "read2", "read7", "read3", 
                                                       "read4", "read5", "read6" };
  std::array<std::string, 7> write_op_handle_names_ = { "write1", "write2", "write7", "write3",
                                                        "write4", "write5", "write6" };

  // Loading of robot info
  hardware_interface::hardware_interface_ret_t get_port();
  hardware_interface::hardware_interface_ret_t get_joint_names();
  hardware_interface::hardware_interface_ret_t get_robot_name();

  // Helper functions
  hardware_interface::hardware_interface_ret_t initialize_vectors();
                                                                                                        
};

}  // namespace abb_egm_hardware

#endif  // ABB_EGM_HARDWARE__ABB_EGM_HARDWARE_HPP_