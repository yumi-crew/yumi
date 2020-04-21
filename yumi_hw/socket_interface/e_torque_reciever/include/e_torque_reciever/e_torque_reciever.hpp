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

  ETorqueReciever(std::string node_name, uint port_left, uint port_right);

  /* Establishes connection with the UDP sockets on the robot controlle of the ABB YuMi*/
  bool establish_connection(int retries = 0);

  /* Blocking call that should be ran in a seperate thread*/
  void recieve_stream(bool debug = false);

  /* issues the stop signal*/
  void stop_stream(){ stop_sign_ = true; };

  void debug_print();

private:
  std::shared_ptr<rclcpp::Node> node_;

  bool connected_;
  bool stop_sign_;
  std::array<double, 14> e_torques_;

  struct SocketInfo
  {
    int comm_socket;
    struct sockaddr_in servaddr;
    bool connected;
    int subsequent_read_fails_counter;
  };

  SocketInfo socket_left_;
  SocketInfo socket_right_;

  std::mutex thread_left_mutex_;
  std::mutex thread_right_mutex_;
  
  // Helper functions
  void parse(std::string data_l, std::string data_r, bool debug_print=false);
};
} // end namepsace socket_interface