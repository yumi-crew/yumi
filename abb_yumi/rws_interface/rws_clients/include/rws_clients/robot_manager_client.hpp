#ifndef RWS_CLIENTS__ROBOT_MANAGER_CLIENT_HPP_
#define RWS_CLIENTS__ROBOT_MANAGER_CLIENT_HPP_

#include <memory>
#include <string>
#include <sstream>
#include <random>

#include <rclcpp/rclcpp.hpp>

#include <rws_clients/visibility_control.h>

#include <yumi_robot_manager_interfaces/srv/stop_egm.hpp>
#include <yumi_robot_manager_interfaces/srv/start_egm.hpp>
#include <yumi_robot_manager_interfaces/srv/is_ready.hpp>


namespace rws_clients
{
class RobotManagerClient 
{
public:

  RWS_CLIENTS_PUBLIC
  RobotManagerClient(std::string name);

  RWS_CLIENTS_PUBLIC
  bool init();

  RWS_CLIENTS_PUBLIC
  bool start_egm();

  RWS_CLIENTS_PUBLIC
  bool stop_egm();

  RWS_CLIENTS_PUBLIC
  bool robot_is_ready();


private:

  std::shared_ptr<rclcpp::Node> node_;
  std::string name_;


};
} // end namespace rws_clients


#endif // RWS_CLIENTS__ROBOT_MANAGER_CLIENT_HPP_