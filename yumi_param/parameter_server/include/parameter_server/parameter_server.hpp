// Copyright 2020 Markus Bjønnes and Marius Nilsen.
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

#pragma once
#include <memory>
#include <string>
#include <iostream>
#include "parameter_server_interfaces/srv/get_all_joints.hpp"
#include "parameter_server_interfaces/srv/get_controller_joints.hpp"
#include "parameter_server_interfaces/srv/get_controller_pid.hpp"
#include "parameter_server_interfaces/srv/get_robot.hpp"
#include "parameter_server_interfaces/srv/get_port.hpp"
#include "rclcpp/rclcpp.hpp"

namespace parameter_server 
{

using GetAllJoints = parameter_server_interfaces::srv::GetAllJoints;
using GetControllerJoints = parameter_server_interfaces::srv::GetControllerJoints;
using GetRobot = parameter_server_interfaces::srv::GetRobot;
using GetControllerPid = parameter_server_interfaces::srv::GetControllerPid;
using GetPort = parameter_server_interfaces::srv::GetPort;
using namespace std::chrono_literals;

class ParameterServer : public rclcpp::Node
{
public:
  explicit ParameterServer( const rclcpp::NodeOptions & options = ( 
                            rclcpp::NodeOptions()
                            .allow_undeclared_parameters(true)
                            .automatically_declare_parameters_from_overrides(true)));

  void load_parameters(const std::string & yaml_config_file);
  void load_parameters(const std::string & key, const std::string & value);
		
private:
  rclcpp::Service<GetAllJoints>::SharedPtr get_all_joints_srv_;
  rclcpp::Service<GetControllerJoints>::SharedPtr get_controller_joints_srv_;
  rclcpp::Service<GetRobot>::SharedPtr get_robot_srv_;
  rclcpp::Service<GetControllerPid>::SharedPtr get_controller_pid_srv_;
  rclcpp::Service<GetPort>::SharedPtr get_port_srv;

  void handle_GetAllJoints(const std::shared_ptr<rmw_request_id_t> request_header,
                            const std::shared_ptr<GetAllJoints::Request> request,
                            const std::shared_ptr<GetAllJoints::Response> response);

  void handle_GetControllerJoints(const std::shared_ptr<rmw_request_id_t> request_header,
                                  const std::shared_ptr<GetControllerJoints::Request> request,
                                  const std::shared_ptr<GetControllerJoints::Response> response);

  void handle_GetRobot(const std::shared_ptr<rmw_request_id_t> request_header,
                        const std::shared_ptr<GetRobot::Request> request,
                        const std::shared_ptr<GetRobot::Response> response);

  void handle_GetControllerPid(const std::shared_ptr<rmw_request_id_t> request_header,
                                const std::shared_ptr<GetControllerPid::Request> request,
                                const std::shared_ptr<GetControllerPid::Response> response);
  
  void handle_GetPort(const std::shared_ptr<rmw_request_id_t> request_header,
                      const std::shared_ptr<GetPort::Request> request,
                      const std::shared_ptr<GetPort::Response> response);
			
};

}  // namespace parameter_server
