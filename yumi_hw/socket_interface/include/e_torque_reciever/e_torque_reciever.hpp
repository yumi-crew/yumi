#include <rclcpp/rclcpp.hpp>
#include <stdlib.h>
#include <unistd.h>
#include <thread>

namespace socket_interface
{
class ETorqueReciever
{
public:

  ETorqueReciever(std::string port_left, std::string port_right);

private:
  std::shared_ptr<rclcpp::Node> node_;
};
} // end namepsace socket_interface