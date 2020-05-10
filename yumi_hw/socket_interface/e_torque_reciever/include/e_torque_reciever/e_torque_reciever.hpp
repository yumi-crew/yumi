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

#include <rclcpp/rclcpp.hpp>
#include <stdlib.h>
#include <unistd.h>
#include <thread>
#include <netinet/ip.h>
#include <arpa/inet.h>
#include <semaphore.h>

namespace socket_interface
{
class ETorqueReciever
{
public:
  ETorqueReciever(std::string node_name, std::string robot_ip, uint port_left, uint port_right);

  /** 
   * Establishes connection with the UDP servers on the ABB YuMi. 
   * 
   * \param retries number of allowed attempts at forming the connections.
   * \return true upon succesfull connection. 
   */
  bool connect(int num_retries = 0);

  /** 
   * Terminates the connection to the UDP servers. 
   * \return true if both sockets close succesfully.
   */ 
  bool disconnect();

  /** 
   * Starts two streams, recieving motor torques from each arm of the ABB YuMi. 
   * 
   * The call is blocking and will only exit upon recieved stop signal or loss of connection with the ABB YuMi. It is
   * recommended to call this method in a seperate thread.
   */
  void start_streams(bool debug = false);

  /* Stops the two channels streaming motor torques. The connection is not terminated. */
  void stop_streams(){ stop_sign_ = true; };

  void debug_print();

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::array<double, 14> e_torques_;
  bool connected_ = false;
  bool stop_sign_ = false;
  int allowed_consecutive_read_fails_ = 5;
  char buffer_l_[100];
  char buffer_r_[100];
  
  struct SocketInfo
  {
    int comm_socket;
    struct sockaddr_in servaddr;
    bool connected;
    int consecutive_read_fails_counter;
  };

  SocketInfo socket_left_;
  SocketInfo socket_right_;
  std::mutex thread_left_mutex_;
  std::mutex thread_right_mutex_;
  
  void parse(bool debug_print=false);
};

} // end namepsace socket_interface