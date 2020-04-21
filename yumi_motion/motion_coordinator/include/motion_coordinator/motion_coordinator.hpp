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

#include <moveit2_wrapper/moveit2_wrapper.hpp>
#include <rws_clients/robot_manager_client.hpp>
#include <rws_clients/grip_client.hpp>

namespace motion_coordinator
{

class MotionCoordinator 
{
public:
  MotionCoordinator(std::string node_name);

  /* Initialize the MotionCoordinator. */
  bool init();

  /** Activates YuMi and launches the planning scene. 
   * 
   * @return true if YuMi is ready to be operated.
   */
  bool activate();

  /* Stops EGM transmissions and shuts down YuMi's motors. */
  void terminate_egm_session();

 /**
   * Moves the last_link of the planning component to the desired pose.
   * 
   * @param pose desired pose. [quaternions]
   * @param eulerzyx flag indicating if ZYX-Euler angles are used instead for quaternions.
   * @param num_retries number of allowed attempts at planning a trajectory.
   * @param visualize flag indicating whether the generated trajectory should be visualized before execution.
   *                  Visualization is only available for blocking motion.
   * @param blocking flag indicating if the function call should be blocking.
   * @param replan flag indicating whether the motion should replan upon changes in the planning scene during motion.
   *               Replanning is only available for blocking motion.
   * 
   * @return true if the planner was able to plan to the goal.
   */
  void move_to_pose(std::string planning_component, std::vector<double> pose, bool eulerzyx=false, int num_retries=0, 
                    bool visualize=false, bool blocking=true, bool replan=false);

  /* Randomly side-shifts the bin. Returns the new position. */
  std::vector<double> random_move_bin(std::vector<double> old_pos);

  /* Return whether a planning_component is moving. */
  bool planning_component_in_motion(std::string planning_component);

  std::shared_ptr<rclcpp::Node> get_node() { return node_; };

private:
  std::string node_name_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rws_clients::RobotManagerClient> yumi_manager_;
  std::shared_ptr<rws_clients::GripClient> left_gripper_;
  std::shared_ptr<rws_clients::GripClient> right_gripper_;
  std::shared_ptr<moveit2_wrapper::Moveit2Wrapper> moveit2_wrapper_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;

  bool robot_ready_ = false;
  double replan_delay_ = 1.0;
  std::mutex should_replan_mutex_;
  std::vector<double> home_l = {0.0, -2.26, 2.35, 0.52, 0.0, 0.52, 0.0};
  std::vector<double> home_r = {0.0, -2.26, -2.35, 0.52, 0.0, 0.52, 0.0};
  
  /* Stops the trajectory controller of a registered planning_component. */
  void stop_motion(std::string planning_component);

  /* Start the trajectory controller of a registered planning_component. */
  void allow_motion(std::string planning_component);

  void joint_state_callback(sensor_msgs::msg::JointState::UniquePtr msg);
};

} // namespace motion_coordinator