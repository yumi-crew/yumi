#ifndef YUMI_ROBOT_MANAGER__YUMI_ROBOT_MANAGER_HPP_
#define YUMI_ROBOT_MANAGER__YUMI_ROBOT_MANAGER_HPP_

#include <string>
#include <unistd.h>
#include <boost/algorithm/string.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>

#include <yumi_robot_manager/visibility_control.h>

#include <yumi_robot_manager_interfaces/srv/stop_egm.hpp>
#include <yumi_robot_manager_interfaces/srv/start_egm.hpp>
#include <yumi_robot_manager_interfaces/srv/is_ready.hpp>


namespace yumi_robot_manager
{
  using StopEgm  = yumi_robot_manager_interfaces::srv::StopEgm;
  using StartEgm = yumi_robot_manager_interfaces::srv::StartEgm;
  using IsReady  = yumi_robot_manager_interfaces::srv::IsReady;

class YumiRobotManager
{
public:

  YUMI_ROBOT_MANAGER_PUBLIC
  YumiRobotManager(const std::string &name, const std::string &ip_address);
  
  YUMI_ROBOT_MANAGER_PUBLIC
  bool init();

  YUMI_ROBOT_MANAGER_PUBLIC
  bool start_state_machine();

  YUMI_ROBOT_MANAGER_PUBLIC
  bool go_to_state(std::string mode);

  YUMI_ROBOT_MANAGER_PUBLIC
  bool run_setup_tests();  

  YUMI_ROBOT_MANAGER_PUBLIC
  bool stop_egm();

  YUMI_ROBOT_MANAGER_PUBLIC
  void spin();

 
private:

  std::string name_;
  std::shared_ptr<rclcpp::Node> node_;
  bool first_execution_ = true;
  std::string requested_state_;
  bool is_ready_ = false;

  std::string task_L_ = "T_ROB_L";    /// Fix
  std::string task_R_ = "T_ROB_R";

  std::string mech_unit_L_ = "ROB_L"; // Fix
  std::string mech_unit_R_ = "ROB_R";


  // Helper functions 
  bool get_configuration_data();
  bool test_grippers();
  void busy_wait_until_idle();
  void wait_for_gripper_to_finish_motion();


  // Service server
  rclcpp::Service<StopEgm>::SharedPtr stop_egm_srv_;
  rclcpp::Service<StartEgm>::SharedPtr start_egm_srv_;
  rclcpp::Service<IsReady>::SharedPtr is_ready_srv_;


  void handle_StopEgm(const std::shared_ptr<rmw_request_id_t> request_header,
                      const std::shared_ptr<StopEgm::Request> request,
                      const std::shared_ptr<StopEgm::Response> response);

  void handle_StartEgm(const std::shared_ptr<rmw_request_id_t> request_header,
                       const std::shared_ptr<StartEgm::Request> request,
                       const std::shared_ptr<StartEgm::Response> response);

  void handle_IsReady( const std::shared_ptr<rmw_request_id_t> request_header,
                       const std::shared_ptr<IsReady::Request> request,
                       const std::shared_ptr<IsReady::Response> response);
  


};

} //namespace yumi_robot_manager


#endif  // YUMI_ROBOT_MANAGER__YUMI_ROBOT_MANAGER_HPP_