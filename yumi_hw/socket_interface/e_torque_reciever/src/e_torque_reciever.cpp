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

  // Left arm UDP socket
  socket_left_.comm_socket = socket(AF_INET, SOCK_STREAM, 0);
  socket_left_.servaddr.sin_family = AF_INET;
  socket_left_.servaddr.sin_port = htons(port_left);
  inet_pton(AF_INET, robot_ip.c_str(), &(socket_left_.servaddr.sin_addr));
  socket_left_.consecutive_read_fails_counter = 0;
  socket_left_.connected = false;

  // Right arm UDP socket
  socket_right_.comm_socket = socket(AF_INET, SOCK_STREAM, 0);
  socket_right_.servaddr.sin_family = AF_INET;
  socket_right_.servaddr.sin_port = htons(port_right);
  inet_pton(AF_INET, robot_ip.c_str(), &(socket_right_.servaddr.sin_addr));
  socket_right_.consecutive_read_fails_counter = 0;
  socket_right_.connected = true;
}


bool ETorqueReciever::connect(int num_retries)
{
  if((connect(socket_right_.comm_socket,(struct sockaddr*)&(socket_right_.servaddr),sizeof(socket_right_.servaddr))==0) 
      && 
    (connect(socket_left_.comm_socket,(struct sockaddr*)&(socket_left_.servaddr),sizeof(socket_left_.servaddr))==0))
  {
    socket_left_.connected = true;
    socket_right_.connected = true;
    connected_ = true;
  }

  int retries_left = num_retries;
  while(!connected_) 
  {
    if(retries_left)
    {
      if((connect(socket_left_.comm_socket,(struct sockaddr*)&(socket_left_.servaddr),sizeof(socket_left_.servaddr))==0) 
          && 
         (connect(socket_right_.comm_socket,(struct sockaddr*)&(socket_right_.servaddr),sizeof(socket_right_.servaddr))==0))
      {
        socket_left_.connected = true;
        socket_right_.connected = true;
        connected_ = true;
      }
    }
    else
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Unable to establish socket connection with YuMi." 
      << "\nleft_socket connected: " << std::boolalpha << socket_left_.connected 
      << "  right_socket connected: " << std::boolalpha << socket_left_.connected);
      return false;
    }
    --retries_left; 
  }
  return true;
}


void ETorqueReciever::start_streams(bool debug)
{
  std::cout << " ** entering start_streams()" << std::endl;
   
  std::cout << " ** before spinning out right thread" << std::endl;
  // Right socket thread
  std::thread right_thread([this]()
  {
    char buf[100]; 
    int ret_r;
    while (!stop_sign_ && connected_)
    {
      bzero(buf, sizeof(buf));
      ret_r = recv(socket_right_.comm_socket, buf, sizeof(buf), 0);

      if(ret_r >= 0) 
      {
        thread_right_mutex_.lock();
        bzero(buffer_r_, sizeof(buffer_r_));
        buffer_r_ = buf;
        thread_right_mutex_.unlock();
        socket_right_.consecutive_read_fails_counter = 0;
      }
      else
      {
        if(socket_right_.consecutive_read_fails_counter < allowed_consecutive_read_fails_)
        {
          socket_right_.consecutive_read_fails_counter++;
          continue;
        }
        else 
        {
          RCLCPP_ERROR_STREAM(node_->get_logger(), "Right socket failed to read " << allowed_consecutive_read_fails_ 
            << " consecutive attempts. Stopping right stream.");
          break;
        }
      }
    }
  });


  std::cout << " ** before spinning out left thread" << std::endl;
  // Left socket thread
  std::thread left_thread([this]()
  {
    char buf[100];
    int ret_l;
    while (!stop_sign_ && connected_)
    {
      bzero(buf, sizeof(buf));
      ret_l = recv(socket_left_.comm_socket, buf, sizeof(buf), 0);
      
      if(ret_l >= 0) 
      {
        thread_left_mutex_.lock();
        bzero(buffer_l_, sizeof(buffer_l_));
        buffer_l_ = buf;
        thread_left_mutex_.unlock();
        socket_left_.consecutive_read_fails_counter = 0;
      }
      else
      {
        if(socket_left_.consecutive_read_fails_counter < 5)
        {
          socket_left_.consecutive_read_fails_counter++;
          continue;
        }
        else 
        {
          RCLCPP_ERROR_STREAM(node_->get_logger(), "Left socket failed to read " << allowed_consecutive_read_fails_ 
            << " consecutive attempts. Stopping left stream.");
          break;
        }
      }
    }
  });

  std::cout << " ** before while (parsing)" << std::endl;
  // Parse
  while(!stop_sign_ && connected_)
  {
    thread_left_mutex_.lock();
    thread_right_mutex_.lock();
    parse(debug);    
    thread_left_mutex_.unlock();
    thread_right_mutex_.unlock();
    if(stop_sign_) break;
  }

  left_thread.join();
  right_thread.join();
  RCLCPP_INFO_STREAM(node_->get_logger(), "Threads stopped successfully.");
}


void ETorqueReciever::parse(bool debug)
{
  size_t pos = 0;
  int i = 0;
  std::string token;
  std::string data_left = buffer_l_;
  std::string data_right = buffer_r_;

  // Parsing the first part of data_left, containing the e_torques of the left arm
  while ((pos = data_left.find(";")) != std::string::npos)
  {
    token = data_left.substr(0, pos);
    e_torques_[i] = std::stod(token);
    data_left.erase(0, pos + 1);
    i++;
  }
  
  // Parsing the first part of data_right, containing the e_torques of the right arm
  i++; pos = 0;
  while ((pos = data_right.find(";")) != std::string::npos)
  {
    token = data_right.substr(0, pos);
    e_torques_[i] = std::stod(token);
    data_right.erase(0, pos + 1);
    i++;
  }

  if(debug) debug_print();
}


void ETorqueReciever::debug_print()
{
  std::cout << "{" << std::endl;
  std::cout << "  Left arm" << std::endl;
  for(int j = 0; j<7; j++) 
  {
    if(j == 2) std::cout << "\tjoint 7: " << e_torques_[j] << std::endl;
    else if(j > 2)  std::cout << "\tjoint " << j << ": " << e_torques_[j] << std::endl;
    else std::cout << "\tjoint " << j+1 << ": " << e_torques_[j] << std::endl;
  }

  std::cout << "  Right arm" << std::endl;
  for(int j = 7; j<14; j++) 
  {
    if(j == 9) std::cout << "\tjoint 7: " << e_torques_[j] << std::endl;
    else if(j > 9)  std::cout << "\tjoint " << j-7 << ": " << e_torques_[j] << std::endl;
    else std::cout << "\tjoint " << j-7+1 << ": " << e_torques_[j] << std::endl;
  }
  std::cout << "}" << std::endl;
}


void ETorqueReciever::disconnect()
{
  bool success_left, success_right = true;
  if(close(socket_left_.comm_socket != 0)
  {
    success_left = false;
    std::cout << "[ERROR] An error occured while disconnecting left socket." << std::endl; 
  }
  if(close(socket_right_.comm_socket != 0)
  {
    success_right = false;
    std::cout << "[ERROR] An error occured while disconnecting right socket." << std::endl;
  }
  return (success_left && success_right);
}

} // end namespace socket_interface