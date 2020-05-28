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
#include <sensor_msgs/msg/joint_state.hpp>

namespace socket_interface
{
class ETorqueReciever
{
public:
  ETorqueReciever(std::string node_name, std::string robot_ip, uint port);

  /** 
   * Establishes connection with a TCP server on the robot controller of YuMi. 
   * 
   * @param retries number of allowed attempts at forming the connections.
   * @return bool indicating if the connection was successfull. 
   */
  bool establish_connection(int num_retries = 0);

  /** 
   * Terminates the connection to the TCP server. 
   * 
   * @return bool indicating if the socket closed succesfully.
   */ 
  bool disconnect();

  /** 
   * @brief Opens the stream, recieving the sent motor torques cause by an external load.
   * 
   * The call is blocking and will only exit upon recieved stop signal or loss of connection with YuMi. It is
   * recommended to call this method in a seperate thread.
   */
  void start_streams(bool debug = false);

  /* Stops the the stream of motor torques. The connection is not terminated. */
  void stop_streams(){ stop_sign_ = true; };

  void debug_print();

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::array<double, 7> e_torques_;
  std::string namespace_;

  bool connected_ = false;
  bool stop_sign_ = false;
  int rate_ = 500; // Publishing rate
  int allowed_consecutive_read_fails_ = 5;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> publisher_;
  
  struct SocketInfo
  {
    int comm_socket;
    struct sockaddr_in servaddr;
    bool connected;
    int consecutive_read_fails_counter;
  };

  SocketInfo socket_; 
  void parse(std::string data_, bool debug_print=false);
};

} // end namepsace socket_interface