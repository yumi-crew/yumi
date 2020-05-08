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
#include <moveit2_wrapper/table_monitor.hpp>
#include <rws_clients/robot_manager_client.hpp>
#include <rws_clients/grip_client.hpp>
#include <kdl_wrapper/kdl_wrapper.h>
#include <rclcpp/parameter.hpp>

namespace motion_coordinator
{

class MotionCoordinator 
{
public:
  MotionCoordinator(std::string node_name);

  /* Initialize the MotionCoordinator. */
  bool init();

  /** 
   * Activates YuMi and launches the planning scene. 
   * 
   * @return true if YuMi is ready to be operated.
   */
  bool activate();

  /* Stops EGM transmissions and shuts down YuMi's motors. */
  void terminate_egm_session();

 /**
  * Moves the registered end-effector link of the planning component to the desired pose.
  * 
  * @param pose desired pose. [quaternions]
  * @param eulerzyx flag indicating if ZYX-Euler angles [degrees] are used instead for quaternions.
  * @param num_retries number of allowed attempts at planning a trajectory.
  * @param visualize flag indicating whether the generated trajectory should be visualized before execution.
  *                  Visualization is only available when no other planning_component is in motion.
  * @param blocking flag indicating if the function call should be blocking.
  * @param replan flag indicating whether the motion should replan upon changes in the planning scene during motion.
  *               Replanning is only available for blocking motion.
  */
  void move_to_pose(std::string planning_component, std::vector<double> pose, bool eulerzyx=false, int num_retries=0, 
                    bool visualize=false, bool blocking=true, bool replan=false);

  /**
  * Moves the registered end-effector link of the planning component to a registered object.
  * 
  * @param object_id registered object.
  * @param hover_height the desired stopping-height over the object.
  * @param num_retries number of allowed attempts at planning a trajectory.
  * @param visualize flag indicating whether the generated trajectory should be visualized before execution.
  *                  Visualization is only available when no other planning_component is in motion.
  * @param blocking flag indicating if the function call should be blocking.
  * @param replan flag indicating whether the motion should replan upon changes in the planning scene during motion.
  *               Replanning is only available for blocking motion.
  */
  void move_to_object(std::string planning_component, std::string object_id, int num_retries, double hover_height,
                      bool visualize=false, bool blocking=true, bool replan=false);
  
  /** 
   * Moves the registered end-effector link of the planning component in a straight-line to the desired pose. 
   * 
   * @param pose desired pose. [quaternions]
   * @param eulerzyx flag indicating if ZYX-Euler angles [degrees] are used instead for quaternions.
   * @param num_retries number of allowed attempts of finding minimum \percentage linear path.
   * @param visualize flag indicating whether the generated trajectory should be visualized before execution.
   *                  Visualization is only available when no other planning_component is in motion.
   * @param blocking flag indicating if the function call should be blocking.
   * @param collision_checking flag indicating whether the path should be verified to be collision free.
   * @param speed_scaling scaling factor used to scale the velocity of the trajectory.
   * @param percentage desired 'linearity' of the computed cartesian straight-line path, 1 indicate a perfectly
   *                   straight line.
   */
  bool linear_move_to_pose(std::string planning_component, std::vector<double> pose, bool eulerzyx, int num_retries,  
                           bool visualize=false, bool blocking=true, bool collision_checking=true, 
                           double percentage=1, double speed_scaling=1, double acc_scaling=1); 
                          
  /** 
   * Moves the registered end-effector link of the planning component in a straight-line in to a registered object.
   * 
   * @param object_id registered object.
   * @param hover_height the desired stopping-height over the object.
   * @param num_retries number of allowed attempts of finding minimum \e percentage linear path.
   * @param visualize flag indicating whether the generated trajectory should be visualized before execution.
   *                  Visualization is only available when no other planning_component is in motion.
   * @param blocking flag indicating if the function call should be blocking.
   * @param collision_checking flag indicating whether the path should be verified to be collision free.
   * @param speed_scaling scaling factor used to scale the velocity of the trajectory.
   * @param percentage desired 'linearity' of the computed cartesian straight-line path, 1 indicate a perfectly
   *                   straight line.
   */
  bool linear_move_to_object(std::string planning_component, std::string object_id, int num_retries,double hover_height,
                             bool visualize=false, bool blocking=true, bool collision_checking=true, 
                             double percentage=1, double speed_scaling=1, double acc_scaling=1);

  /** 
   * Moves the planning component to its registered home configuration. 
   * 
   * @param num_retries number of allowed attempts at planning a trajectory.
   * @param visualize flag indicating whether the generated trajectory should be visualized before execution.
   *                  Visualization is only available when no other planning_component is in motion.
   * @param blocking flag indicating if the function call should be blocking.
   * @param replan flag indicating whether the motion should replan upon changes in the planning scene during motion.
   *               Replanning is only available for blocking motion.
   */
  void move_to_home(std::string planning_component, int num_retries=2, bool visualize=false, bool blocking=true,
                    bool replan=false);

  /**
   * Opens the gripper of the planning component, moves to the gripping point in a straight line, closes the gripper and 
   * returns to the starting pose in a straight line.
   * 
   * \return true if the object is estimated to be successfully picked.
   */
  bool pick_object(std::string planning_component, std::string object_id, int num_retries, double hover_height, bool blocking, 
                   bool visualize, double percentage);
  
  /**
   * Moves the planning component in a straight line to the given pose, opens the gripper and returns to the starting 
   * pose in a straight line.
   * 
   * \return true if the object is estimated to be successfully placed.
   */
  bool place_at_object(std::string planning_component, std::string object_id, int num_retries, double hover_height, bool blocking, 
                       bool visualize, double percentage);

  /* Return whether a planning_component is moving. */
  bool planning_component_in_motion(std::string planning_component);

  /* Stops the execution a planning component's trajectory. */
  void stop(std::string planning_component);

  /* Add a registered object to the planning scene. */
  void add_object(std::string object_id, std::vector<double> pose, bool eulerzyx, std::vector<float> rgba = {});

  /* Removes a registered object. */
  void remove_object(std::string object_id);

  /* Moves a registered object. Pose must be given using quaternions. */
  void move_object(std::string object_id, std::vector<double> pose);

  /* Checks if a registered object is present in the planning scene. */
  bool object_present(std::string object_id){ return !table_monitor_->find_object(object_id).empty(); }

   /* Randomly side-shifts an registered object. Returns the new position. */
  std::vector<double> random_move_object(std::string object_id, double side_shift);
 
  bool should_stop(){ return should_stop_; }
  void grip_in(std::string gripper, bool blocking);
  void grip_out(std::string gripper, bool blocking);
  std::shared_ptr<rclcpp::Node> get_node() { return node_; };

private:
  std::string node_name_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rws_clients::RobotManagerClient> yumi_manager_;
  std::shared_ptr<rws_clients::GripClient> left_gripper_;
  std::shared_ptr<rws_clients::GripClient> right_gripper_;
  std::shared_ptr<moveit2_wrapper::Moveit2Wrapper> moveit2_wrapper_;
  std::shared_ptr<moveit2_wrapper::TableMonitor> table_monitor_;
  std::shared_ptr<KdlWrapper> kdl_wrapper_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;

  bool should_stop_ = false;
  bool robot_ready_ = false;
  double replan_delay_ = 1.0;
  double speed_scale_ = 1.0;
  double acc_scale_ = 1.0;
  std::mutex should_replan_mutex_;

  /* Stops the trajectory controller of a registered planning_component. */
  void stop_motion(std::string planning_component);

  /* Start the trajectory controller of a registered planning_component. */
  void allow_motion(std::string planning_component);

  /* Returns a position randomly shifted side_shift in the X or Y direction. */
  std::vector<double> random_nearby_position(std::vector<double> old_position, double side_shift);

  /* Returns a random orientation between l_limit and u_limit degrees away from current orientation. Orientation
     represented using ZYX-Euler angles (degrees) */
  std::vector<double> random_orientation(double l_limit, double u_limit);

  /* Returns a random nearby pose. Orientation given using ZYX-Euler angles. [degrees]*/
  std::vector<double> get_random_nearby_pose(std::vector<double> old_pose, double side_shift, double l_limit, 
                                             double u_limit);

  /* Finds a configuration giving the same end-effector pose. */
  std::vector<double> equivalent_state(std::string planning_component, std::vector<double> pose, bool eulerzyx); 

  void print_matrix(Eigen::Matrix4d mat);
  bool gripper_contain_object(std::string planning_component);
  void joint_state_callback(sensor_msgs::msg::JointState::UniquePtr msg);
};

} // namespace motion_coordinator