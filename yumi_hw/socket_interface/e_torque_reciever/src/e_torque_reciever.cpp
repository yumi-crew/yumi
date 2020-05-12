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
  
ETorqueReciever::ETorqueReciever(std::string node_name, std::string robot_ip, uint port_left, uint port_right)
:
connected_{false}, stop_sign_{false}
{
  node_ = std::make_shared<rclcpp::Node>(node_name);
  publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("/r/external_joint_torques", 10);

  // Right arm UDP socket
  socket_right_.comm_socket = socket(AF_INET, SOCK_STREAM, 0);
  socket_right_.servaddr.sin_family = AF_INET;
  socket_right_.servaddr.sin_port = htons(port_right);
  inet_pton(AF_INET, robot_ip.c_str(), &(socket_right_.servaddr.sin_addr));
  socket_right_.consecutive_read_fails_counter = 0;
  socket_right_.connected = true;
}


bool ETorqueReciever::establish_connection(int num_retries)
{
  if(connect(socket_right_.comm_socket,(struct sockaddr*)&(socket_right_.servaddr),sizeof(socket_right_.servaddr))==0) 
  {
    socket_right_.connected = true;
    connected_ = true;
  }

  int retries_left = num_retries;
  while(!connected_) 
  {
    if(retries_left)
    {
      if(connect(socket_right_.comm_socket,(struct sockaddr*)&(socket_right_.servaddr),sizeof(socket_right_.servaddr))==0)
      {
        socket_right_.connected = true;
        connected_ = true;
      }
    }
    else
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Unable to establish socket connection with YuMi." 
      << "  right_socket connected: " << std::boolalpha << socket_right_.connected);
      return false;
    }
    --retries_left; 
  }
  return true;
}


void ETorqueReciever::start_streams(bool debug)
{
  std::cout << " ** entering start_streams()" << std::endl;
  char buf_r[100]; 
   
  // std::cout << " ** before spinning out right thread" << std::endl;
  // // Right socket thread
  // std::thread right_thread([this, &buf_r]()
  // {
  //   int ret_r;
  //   rclcpp::WallRate loop(rate_);
  //   while (!stop_sign_ && connected_)
  //   {
  //     ret_r = read(socket_right_.comm_socket, buf_r, sizeof(buf_r));
  //     if(ret_r >= 0) 
  //     {
  //       socket_right_.consecutive_read_fails_counter = 0;
  //     }
  //     else
  //     {
  //       if(socket_right_.consecutive_read_fails_counter < allowed_consecutive_read_fails_)
  //       {
  //         socket_right_.consecutive_read_fails_counter++;
  //         continue;
  //       }
  //       else 
  //       {
  //         RCLCPP_ERROR_STREAM(node_->get_logger(), "Right socket failed to read " << allowed_consecutive_read_fails_ 
  //           << " consecutive attempts. Stopping right stream.");
  //         break;
  //       }
  //     }
  //     loop.sleep();
  //   }
  // });


  // Parse
  rclcpp::WallRate loop(rate_);
  while(!stop_sign_ && connected_)
  {
    read(socket_right_.comm_socket, buf_r, sizeof(buf_r));
    parse(buf_r, debug);    
    if(stop_sign_)
    {
      std::cout << "Aborting streams, stop_sign_:  "<< stop_sign_ << " , connected_: " << connected_ << std::endl; 
      break;
    }
    loop.sleep();
  }
  //right_thread.join();
  //RCLCPP_INFO_STREAM(node_->get_logger(), "Threads stopped successfully.");
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
  std::cout << "  Right arm" << std::endl;
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
  bool success_right = true;
  if(close(socket_right_.comm_socket != 0))
  {
    success_right = false;
    std::cout << "[ERROR] An error occured while disconnecting right socket." << std::endl;
  }
  return success_right;
}

} // end namespace socket_interface