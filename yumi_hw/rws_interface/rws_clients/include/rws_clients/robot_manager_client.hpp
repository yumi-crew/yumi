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

#pragma once

#include <memory>
#include <string>
#include <sstream>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <rws_clients/visibility_control.h>
#include <yumi_robot_manager_interfaces/srv/stop_egm.hpp>
#include <yumi_robot_manager_interfaces/srv/start_egm.hpp>
#include <yumi_robot_manager_interfaces/srv/is_ready.hpp>
#include <yumi_robot_manager_interfaces/srv/stop_motors.hpp>


namespace rws_clients
{

class RobotManagerClient 
{
public:
  RWS_CLIENTS_PUBLIC
  RobotManagerClient(std::shared_ptr<rclcpp::Node> node);

  RWS_CLIENTS_PUBLIC
  bool init();

  RWS_CLIENTS_PUBLIC
  bool start_egm();

  RWS_CLIENTS_PUBLIC
  bool stop_egm();

  RWS_CLIENTS_PUBLIC
  bool robot_is_ready();

  RWS_CLIENTS_PUBLIC
  bool stop_motors();

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::string name_;

};

} // end namespace rws_clients

