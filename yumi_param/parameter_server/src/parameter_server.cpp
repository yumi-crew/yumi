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

#include "parameter_server/parameter_server.hpp"
#include "parameter_server/yaml_parser.hpp"
#include <algorithm>
#include <string>
#include <sstream>

namespace parameter_server 
{

bool to_bool(std::string str) 
{
  std::transform(str.begin(), str.end(), str.begin(), ::tolower);
  std::istringstream is(str);
  bool b;
  is >> std::boolalpha >> b;
  return b;
}

using namespace std::placeholders;

ParameterServer::ParameterServer(const rclcpp::NodeOptions & options)
: 
rclcpp::Node("parameter_server", options)
{
  auto fcn1 = std::bind(&ParameterServer::handle_GetRobot, this, _1, _2, _3);
  get_robot_srv_ = this->create_service<GetRobot>("GetRobot", fcn1, rmw_qos_profile_services_default);
		
  auto fcn2 = std::bind(&ParameterServer::handle_GetPort, this, _1, _2, _3);
  get_port_srv = this->create_service<GetPort>("GetPort", fcn2, rmw_qos_profile_services_default);

  auto fcn3 = std::bind(&ParameterServer::handle_GetAllJoints, this, _1, _2, _3);
  get_all_joints_srv_ = this->create_service<GetAllJoints>("GetAllJoints", fcn3, rmw_qos_profile_services_default);

  auto fcn4 = std::bind(&ParameterServer::handle_GetControllerPid, this, _1, _2, _3);
  get_controller_pid_srv_ = this->create_service<GetControllerPid>("GetControllerPid", fcn4, 
                                                                   rmw_qos_profile_services_default);

  auto fcn5 = std::bind(&ParameterServer::handle_GetControllerJoints, this, _1, _2, _3);
  get_controller_joints_srv_ = this->create_service<GetControllerJoints>("GetControllerJoints", fcn5,
																																	       rmw_qos_profile_services_default);		
}


void ParameterServer::handle_GetAllJoints(const std::shared_ptr<rmw_request_id_t> request_header,
                                          const std::shared_ptr<GetAllJoints::Request> request,
                                          const std::shared_ptr<GetAllJoints::Response> response) 
{
  (void) request_header;
  auto param_list = this->list_parameters({"." + request->robot}, 10);
  std::vector<std::string> joints = {};
  
  for (auto &param : param_list.names) 
  {
    if (param.find("joints") != std::string::npos) 
    {
      auto jointParam = this->get_parameter(param);
      joints.push_back(jointParam.value_to_string());
      // Change from hardcoding
      if(joints.size() >= 7) break;
    }
  }
  response->joints = joints;
}


void ParameterServer::handle_GetControllerJoints(const std::shared_ptr<rmw_request_id_t> request_header,
                                                  const std::shared_ptr<GetControllerJoints::Request> request,
                                                  const std::shared_ptr<GetControllerJoints::Response> response) 
{
  (void) request_header;
  auto param_list = this->list_parameters({""}, 10);
  std::vector<std::string> joints = {};
  for (auto &param : param_list.names)
  {
    if (param.find(request->controller + ".joints") != std::string::npos) 
    {
      auto jointParam = this->get_parameter(param);
      joints.push_back(jointParam.value_to_string());
    }
  }
  response->joints = joints;
}

    
void ParameterServer::handle_GetRobot( const std::shared_ptr<rmw_request_id_t> request_header,
                                        const std::shared_ptr<GetRobot::Request> request,
                                        const std::shared_ptr<GetRobot::Response> response) 
{
  (void) request_header;
  (void) request;
  auto param_list = this->list_parameters({""}, 10);

  auto paramName  = param_list.names[0]; 
  auto paramNameT = paramName.substr(1, paramName.size() - 1);
  auto iter = std::find(paramNameT.begin(), paramNameT.end(), '.');
  auto dist = std::distance(paramNameT.begin(), iter);

  response->robot = paramNameT.substr(0, dist);
}

		
void ParameterServer::handle_GetControllerPid(const std::shared_ptr<rmw_request_id_t> request_header,
                                              const std::shared_ptr<GetControllerPid::Request> request,
                                              const std::shared_ptr<GetControllerPid::Response> response) 
{
  (void) request_header;
  auto param_list = this->list_parameters({""}, 10);
  std::vector<std::string> joints = {};

  for (auto &param : param_list.names) 
  {
    if (param.find(request->controller + ".pid") != std::string::npos) 
    {
      // Get the specific parameter (p,i,d,i_min,i_max or antiwindup)
      std::vector<size_t> dotPos = {};
      for (size_t i = 0; i < param.size(); i++) 
      {
        if (param[i] == '.') 
        {
          dotPos.push_back(i);
        }
      }
      // If less than 3 dots definitely not the correct parameter, so we continue to the next param
      // This also makes it such that our code below which accesses by index is safe
      if (dotPos.size() < 3) continue;
    
      auto endPos = dotPos.size() - 1;
      auto pidParamName = param.substr(dotPos[endPos] + 1, param.size() - dotPos[endPos] - 1);
      if (pidParamName == "p") 
      {
        auto pidParam = this->get_parameter(param);
        auto pValStr = pidParam.value_to_string();
        RCLCPP_DEBUG(this->get_logger(), "p: %s", pValStr.c_str());
        response->p = std::stod(pValStr);
      } 
      else if (pidParamName == "i") 
      {
        auto pidParam = this->get_parameter(param);
        auto iValStr = pidParam.value_to_string();
        RCLCPP_DEBUG(this->get_logger(), "i: %s", iValStr.c_str());
        response->i = std::stod(iValStr);
      } 
      else if (pidParamName == "d") 
      {
        auto pidParam = this->get_parameter(param);
        auto dValStr = pidParam.value_to_string();
        RCLCPP_DEBUG(this->get_logger(), "d: %s", dValStr.c_str());
        response->d = std::stod(dValStr);
      } 
      else if (pidParamName == "i_min") 
      {
        auto pidParam = this->get_parameter(param);
        auto i_minValStr = pidParam.value_to_string();
        RCLCPP_DEBUG(this->get_logger(), "i_min: %s", i_minValStr.c_str());
        response->i_min = std::stod(i_minValStr);
      } 
      else if (pidParamName == "i_max") 
      {
        auto pidParam = this->get_parameter(param);
        auto i_maxValStr = pidParam.value_to_string();
        RCLCPP_DEBUG(this->get_logger(), "i_max: %s", i_maxValStr.c_str());
        response->i_max = std::stod(i_maxValStr);
      } 
      else if (pidParamName == "antiwindup") 
      {
        auto pidParam = this->get_parameter(param);
        auto antiwindupValStr = pidParam.value_to_string();
        RCLCPP_DEBUG(this->get_logger(), "antiwindup: %s", antiwindupValStr.c_str());
        response->antiwindup = to_bool(antiwindupValStr);
      } 
      else 
      {
        auto pidParam = this->get_parameter(param);
        RCLCPP_WARN(this->get_logger(), "Unknown PID parameter: {%s: %s}", param.c_str(),
                    pidParam.value_to_string().c_str());
      }
    }
  }
}


void ParameterServer::handle_GetPort(const std::shared_ptr<rmw_request_id_t> request_header,
                                     const std::shared_ptr<GetPort::Request> request,
                                     const std::shared_ptr<GetPort::Response> response) 
{
  (void) request_header;
  auto param_list = this->list_parameters({"." + request->robot}, 10);

  for (auto &param : param_list.names)
  {
    if(param.find("port") != std::string::npos)
    {
      std::vector<size_t> dotPos = {};
      for (size_t i = 0; i < param.size(); i++) 
      {	
        if (param[i] == '.') 
        {
          dotPos.push_back(i);
        }
      }
      auto port_param = this->get_parameter(param);
      auto str_port_param = port_param.value_to_string();
      response->port = std::stod(str_port_param);
      break;
    }
    else response->port = 0.0;
  }
}


void ParameterServer::load_parameters(const std::string &yaml_config_file) 
{
  if (yaml_config_file.empty()) 
  {
    throw std::runtime_error("yaml config file path is empty");
  }

  YamlParser parser;
  parser.parse(yaml_config_file);

  auto key_values = parser.get_key_value_pairs();
  for (const auto &pair : key_values) 
  {
    this->set_parameters({rclcpp::Parameter(pair.first, pair.second)});
  }
}

void ParameterServer::load_parameters(const std::string &key, const std::string &value)
{
  this->set_parameters({rclcpp::Parameter(key, value)});
}
      						
} // namespace parameter_server
