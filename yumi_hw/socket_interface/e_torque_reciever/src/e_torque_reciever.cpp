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

#include <e_torque_reciever/e_torque_reciever.hpp>

namespace socket_interface
{
  
ETorqueReciever::ETorqueReciever(std::string node_name, std::string robot_ip, uint port)
:
connected_{false}, stop_sign_{false}
{
  node_ = std::make_shared<rclcpp::Node>(node_name);
  namespace_ = node_->get_namespace();
  publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(namespace_+"/external_joint_torques", 10);

  socket_.comm_socket = socket(AF_INET, SOCK_STREAM, 0);
  socket_.servaddr.sin_family = AF_INET;
  socket_.servaddr.sin_port = htons(port);
  inet_pton(AF_INET, robot_ip.c_str(), &(socket_.servaddr.sin_addr));
  socket_.consecutive_read_fails_counter = 0;
  socket_.connected = true;
}


bool ETorqueReciever::establish_connection(int num_retries)
{
  if(connect(socket_.comm_socket,(struct sockaddr*)&(socket_.servaddr),sizeof(socket_.servaddr))==0) 
  {
    socket_.connected = true;
    connected_ = true;
  }

  int retries_left = num_retries;
  while(!connected_) 
  {
    if(retries_left)
    {
      if(connect(socket_.comm_socket,(struct sockaddr*)&(socket_.servaddr),sizeof(socket_.servaddr))==0)
      {
        socket_.connected = true;
        connected_ = true;
      }
    }
    else
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Unable to establish socket connection with YuMi." 
      << "  right_socket connected: " << std::boolalpha << socket_.connected);
      return false;
    }
    --retries_left; 
  }
  return true;
}


void ETorqueReciever::start_streams(bool debug)
{
  std::cout << " ** entering start_streams()" << std::endl;
  char buf[100]; 
  
  // read-parse loop
  rclcpp::WallRate loop(rate_);
  while(!stop_sign_ && connected_)
  {
    read(socket_.comm_socket, buf, sizeof(buf));
    parse(buf, debug);    
    if(stop_sign_)
    {
      std::cout << "Aborting streams, stop_sign_:  "<< stop_sign_ << " , connected_: " << connected_ << std::endl; 
      break;
    }
    loop.sleep();
  }
}


void ETorqueReciever::parse(std::string data, bool debug)
{
  size_t pos = 0;
  std::string token;

  size_t start = data.find("{");
  data.erase(0, start);  // Filter out junk before vector
  size_t end = data.find("}");

  // If recieved string contain both start symbol "{" and end symbol "}"
  if( (start != std::string::npos) && (end != std::string::npos) && (start<end) )
  {
    // Find start of torque-vector and remove all content before start.
    start = data.find("{");
    data.erase(0, start+1); // "{" will also be removed

    // parse contents {t1;t2;t7;t3;t4;t5;t6;}
    for(int i = 0; i < 7; i++)
    {
      pos = data.find(";");

      // If another ";" cannot be found
      if(pos == std::string::npos) break;                              

      // If next ";" is found after the end of the vector
      if(pos > end) break;

      token = data.substr(0, pos);
      e_torques_[i] = std::stod(token);
      data.erase(0, pos + 1);
    }

    sensor_msgs::msg::JointState msg;
    for(int i = 0; i<7; i++)
    {
      msg.effort.push_back(e_torques_[i]);
    }
    msg.header.stamp = node_->now();
    publisher_->publish(msg);

    if(debug) debug_print();
  }
  else return; // If recieved string doesn't contain both the start-delimiter "{" and the end-delimiter "}", skip.
}


void ETorqueReciever::debug_print()
{
  std::cout << "{" << std::endl;
  int j = 0;
  for(auto e : e_torques_) 
  {
    if(j == 2) std::cout << "\tjoint 7: " << e << std::endl;
    else if(j > 2)  std::cout << "\tjoint " << j << ": " << e << std::endl;
    else std::cout << "\tjoint " << j+1 << ": " << e << std::endl;
    j++;
  }
  std::cout << "}" << std::endl;
}


bool ETorqueReciever::disconnect()
{
  if(close(socket_.comm_socket != 0))
  {
    std::cout << "[ERROR] An error occured while disconnecting the socket." << std::endl;
    return false;
  }
  else return true
}

} // end namespace socket_interface