#ifndef RWS_CLIENTS__GRIP_CLIENT_HPP_
#define RWS_CLIENTS__GRIP_CLIENT_HPP_

/*
 * Action client utilizing grips actions of the Smart Grippers of YuMi.
 */

#include <memory>
#include <inttypes.h>
#include <string>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>
#include <rclcpp_action/rclcpp_action.hpp>

#include <rws_clients/visibility_control.h>

#include <sg_control_interfaces/action/grip.hpp>



namespace rws_clients
{

using Grip = sg_control_interfaces::action::Grip;
using GoalHandleGrip = rclcpp_action::ClientGoalHandle<Grip>;


class GripClient : public rclcpp::Node
{
public:

  RWS_CLIENTS_PUBLIC
  GripClient(std::string name, std::string ns);

  RWS_CLIENTS_PUBLIC
  bool init();
  
  RWS_CLIENTS_PUBLIC
  bool is_goal_done() const;
 
  RWS_CLIENTS_PUBLIC
  void perform_grip(int percentage_closed);
  
 
private:

  rclcpp_action::Client<Grip>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_ = false;
  std::string namespace_;
  void send_goal();
  int percentage_closed_;


  void goal_response_callback(std::shared_future<GoalHandleGrip::SharedPtr> future);

  void feedback_callback(GoalHandleGrip::SharedPtr,  const std::shared_ptr<const Grip::Feedback> feedback);
 
  void result_callback(const GoalHandleGrip::WrappedResult &result);
  


};  
}   // end namespace rws_clients


#endif  // RWS_CLIENTS__GRIP_OUT_USER_HPP_

