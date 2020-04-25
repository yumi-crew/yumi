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

#include <motion_coordinator/motion_coordinator.hpp>

namespace motion_coordinator
{

MotionCoordinator::MotionCoordinator(std::string node_name) : node_name_(node_name)
{}


bool MotionCoordinator::init()
{
  // Create node to be used by Moveit2.
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_ = std::make_shared<rclcpp::Node>(node_name_, "", node_options);

  yumi_manager_ = std::make_shared<rws_clients::RobotManagerClient>("yumi_manager_client");
  left_gripper_ = std::make_shared<rws_clients::GripClient>("grip_client_left", "/l");
  right_gripper_ = std::make_shared<rws_clients::GripClient>("grip_client_right", "/r");
  moveit2_wrapper_ = std::make_shared<moveit2_wrapper::Moveit2Wrapper>(node_);
  joint_state_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>("joint_states", 
    10, std::bind(&MotionCoordinator::joint_state_callback, this, std::placeholders::_1));


  if(!yumi_manager_->init())
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "yumi_manager failed to initialize.");
    return false;
  }
  if(!left_gripper_->init())
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "left_gripper failed to initialize.");
    return false;
  }
  if(!right_gripper_->init())
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "right_gripper failed to initialize.");
    return false;
  }
  if(!moveit2_wrapper_->init())
  {
    RCLCPP_ERROR(node_->get_logger(), "moveit2_wrapper failed to initialize.");
    return false;
  }

  table_monitor_ = std::make_shared<moveit2_wrapper::TableMonitor>(moveit2_wrapper_->get_moveit_cpp());
  if(!table_monitor_->init())
  {
    RCLCPP_ERROR(node_->get_logger(), "table_monitor failed to initialize.");
    return false;
  }

  std::cout << "init successfully " << std::endl;
  return true;
}


bool MotionCoordinator::activate()
{
  while(!yumi_manager_->robot_is_ready())
  {
    sleep(0.1);
  }
  if(!yumi_manager_->start_egm())
  {
    sleep(1);
    if(!yumi_manager_->start_egm()) { return false; }
  }

  // Wait until robot is reporting its state on the the joint_states topic.
  while(!robot_ready_){ sleep(0.1); };
  // Subscription no longer needed.
  joint_state_subscription_.reset(); 
  moveit2_wrapper_->launch_planning_scene();
  
  if(!table_monitor_->activate())
  {
    RCLCPP_ERROR(node_->get_logger(), "table_monitor failed to activate. ");
    return false;
  }
  return true;
}


void MotionCoordinator::terminate_egm_session()
{
  yumi_manager_->stop_egm();
  yumi_manager_->stop_motors();
}


void MotionCoordinator::stop_motion(std::string planning_component)
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  auto planning_components_hash = moveit2_wrapper_->get_planning_components_hash();
  if(planning_component == "both_arms")
  {
    planning_components_hash->at("left_arm").stop_signal_publisher->publish(msg);
    planning_components_hash->at("right_arm").stop_signal_publisher->publish(msg);
    planning_components_hash->at("both_arms").in_motion = false;
  }
  else
  {
    planning_components_hash->at(planning_component).stop_signal_publisher->publish(msg);
    planning_components_hash->at(planning_component).in_motion = false;
  }
}


void MotionCoordinator::allow_motion(std::string planning_component)
{
  std_msgs::msg::Bool msg;
  msg.data = false;
  auto planning_components_hash = moveit2_wrapper_->get_planning_components_hash();
  if(planning_component == "both_arms")
  {
    planning_components_hash->at("left_arm").stop_signal_publisher->publish(msg);
    planning_components_hash->at("right_arm").stop_signal_publisher->publish(msg);
  }
  else
  {
    planning_components_hash->at(planning_component).stop_signal_publisher->publish(msg);
  }
}


void MotionCoordinator::move_to_pose(std::string planning_component, std::vector<double> pose, bool eulerzyx, 
                                     int num_retries, bool visualize, bool blocking, bool replan)
{
  std::cout << "move_to_pose() called for planning component '" << planning_component << "'" <<  std::endl;

  if(replan && !blocking) 
  { 
    RCLCPP_WARN_STREAM(node_->get_logger(), "Replanning is only available for blocking motion."); 
    return;
  }
  auto planning_component_hash = moveit2_wrapper_->get_planning_components_hash();
  std::string ee_link = planning_component_hash->at(planning_component).ee_link;
  
  // If it should replan upon changed planning_scene
  if(replan)
  {
    // Run initial motion non-blocking.
    moveit2_wrapper_->pose_to_pose_motion(planning_component, ee_link, pose,eulerzyx, num_retries, visualize, replan, 
                                          false, speed_scale_);

    while(!moveit2_wrapper_->pose_reached(planning_component, ee_link, pose, eulerzyx))
    {
      if(planning_component_hash->at(planning_component).should_replan)
      {
        stop(planning_component);
        should_replan_mutex_.lock();
        planning_component_hash->at(planning_component).should_replan = false; 
        should_replan_mutex_.unlock();
        moveit2_wrapper_->pose_to_pose_motion(planning_component, ee_link, pose, eulerzyx, num_retries, visualize, 
                                              replan, false, speed_scale_);
      }
    }
    RCLCPP_INFO_STREAM(node_->get_logger(), "Goal pose of planning component '" << planning_component 
      << "' considered reached.");
    planning_component_hash->at(planning_component).in_motion = false;
  }
  else
  {
    moveit2_wrapper_->pose_to_pose_motion(planning_component, ee_link, pose, eulerzyx, num_retries, visualize, replan, 
                                          blocking, speed_scale_);
  }
}


void MotionCoordinator::move_to_object(std::string planning_component, std::string object_id, double hover_height,  
                                       int num_retries, bool visualize, bool blocking, bool replan)
{
  std::cout << "move_to_object() called for planning component '" << planning_component << "'" <<  std::endl;

  if(replan && !blocking) 
  { 
    RCLCPP_WARN_STREAM(node_->get_logger(), "Replanning is only available for blocking motion."); 
    return;
  }
  auto planning_component_hash = moveit2_wrapper_->get_planning_components_hash();
  std::string ee_link = planning_component_hash->at(planning_component).ee_link;

  std::vector<double> pose = table_monitor_->find_object(object_id); 
  if(pose.empty())
  { 
    stop(planning_component);
    RCLCPP_WARN_STREAM(node_->get_logger(), "Can't find object, moving to home.");
    move_to_home(planning_component, num_retries, true, blocking, false);
    return; 
  }
  else apply_gripper_transform(pose, object_id, hover_height); 
  
  // If it should replan upon changed planning_scene
  if(replan)
  {
    // Run initial motion non-blocking.
    moveit2_wrapper_->pose_to_pose_motion(planning_component, ee_link, pose, false, num_retries, visualize, replan, 
                                          false, speed_scale_);

    while(!moveit2_wrapper_->pose_reached(planning_component, ee_link, pose, false))
    {
      if(planning_component_hash->at(planning_component).should_replan)
      {
        stop(planning_component);
        pose = table_monitor_->find_object(object_id); 

        if(pose.empty())
        { 
          stop(planning_component);
          RCLCPP_WARN_STREAM(node_->get_logger(), "Can't find object, moving to home.");
          move_to_home(planning_component, num_retries, true, blocking, false);
          return; 
        }
        else apply_gripper_transform(pose, object_id, hover_height);

        should_replan_mutex_.lock();
        planning_component_hash->at(planning_component).should_replan = false; 
        should_replan_mutex_.unlock();
        moveit2_wrapper_->pose_to_pose_motion(planning_component, ee_link, pose, false, num_retries, visualize, replan, 
                                              false, speed_scale_);
        
      }
    }
    RCLCPP_INFO_STREAM(node_->get_logger(), "Goal pose of planning component '" << planning_component 
      << "' considered reached.");
    planning_component_hash->at(planning_component).in_motion = false;
  }
  else
  {
    moveit2_wrapper_->pose_to_pose_motion(planning_component, ee_link, pose, false, num_retries, visualize, replan, 
                                          blocking, speed_scale_);
  }
}


void MotionCoordinator::linear_move_to_pose(std::string planning_component, std::vector<double> pose, bool eulerzyx, 
                                            bool visualization, bool blocking, double speed_scaling, double percentage)
{
  std::cout << "linear_move_to_pose() called for planning component '" << planning_component << "'" <<  std::endl;

  auto planning_components_hash = moveit2_wrapper_->get_planning_components_hash();
  std::string link = planning_components_hash->at(planning_component).ee_link;
 
  moveit2_wrapper_->cartesian_pose_to_pose_motion(planning_component, link, pose, eulerzyx, visualization, false, 
                                                  blocking, percentage, speed_scaling, 1 );
}


void MotionCoordinator::move_to_home(std::string planning_component, int num_retries, bool visualize, bool blocking, 
                                     bool replan)
{
  std::cout << "move_to_home() called for planning component '" << planning_component << "'" <<  std::endl;

  if(replan && !blocking) 
  { 
    RCLCPP_WARN_STREAM(node_->get_logger(), "Replanning is only available for blocking motion."); 
    return;
  }
  auto planning_component_hash = moveit2_wrapper_->get_planning_components_hash();
  std::vector<double> home =  planning_component_hash->at(planning_component).home_configuration;
  
  // If it should replan upon changed planning_scene
  if(replan)
  {
    // Run initial motion non-blocking.
    moveit2_wrapper_->state_to_state_motion(planning_component, home, num_retries, visualize, replan, false, 
                                            speed_scale_);

    while(!moveit2_wrapper_->state_reached(planning_component, home))
    {
      if(planning_component_hash->at(planning_component).should_replan)
      {
        stop(planning_component);
        moveit2_wrapper_->state_to_state_motion(planning_component, home, num_retries, visualize, replan, false, 
                                                speed_scale_);
        should_replan_mutex_.lock();
        planning_component_hash->at(planning_component).should_replan = false; 
        should_replan_mutex_.unlock();
      }
    }
    RCLCPP_INFO_STREAM(node_->get_logger(), "Goal state of planning component '" << planning_component 
      << "' considered reached.");
    planning_component_hash->at(planning_component).in_motion = false;
  }
  else
  {
    moveit2_wrapper_->state_to_state_motion(planning_component, home, num_retries, visualize, replan, blocking, 
                                            speed_scale_);
  }
}


std::vector<double> MotionCoordinator::random_move_bin(std::vector<double> old_pos)
{ 
  srand((unsigned)time(NULL));
  double rnum = rand()%(12-8)+8; 
  double side_shift = 0;
  if(rnum>10) side_shift += rnum;
  else side_shift -= rnum;
  std::vector<double> new_pose{old_pos[0], old_pos[1]+side_shift/100, old_pos[2], 0, 0, 0, 1};

  table_monitor_->move_object("bin", new_pose);

  auto planning_components_hash = moveit2_wrapper_->get_planning_components_hash();
  should_replan_mutex_.lock();
  planning_components_hash->at("left_arm").should_replan = true;
  planning_components_hash->at("right_arm").should_replan = true;
  planning_components_hash->at("both_arms").should_replan = true;
  should_replan_mutex_.unlock();
  return {new_pose[0], new_pose[1], new_pose[2]};
}


void MotionCoordinator::remove_object(std::string object_id)
{ 
  table_monitor_->remove_object_from_scene(object_id, true); 

  auto planning_components_hash = moveit2_wrapper_->get_planning_components_hash();
  should_replan_mutex_.lock();
  planning_components_hash->at("left_arm").should_replan = true;
  planning_components_hash->at("right_arm").should_replan = true;
  planning_components_hash->at("both_arms").should_replan = true;
  should_replan_mutex_.unlock();
}


void MotionCoordinator::joint_state_callback(sensor_msgs::msg::JointState::UniquePtr msg)
{
  int counter = 14;
  for(auto value : msg->position)
  {
    if(value == 0) --counter;
  }
  if(counter != 14) robot_ready_ = true;
}


bool MotionCoordinator::planning_component_in_motion(std::string planning_component)
{
  auto planning_component_hash = moveit2_wrapper_->get_planning_components_hash();
  return planning_component_hash->at(planning_component).in_motion;
}


void MotionCoordinator::apply_gripper_transform(std::vector<double>& pose, std::string object_id, double hover_height)
{
  KDL::Rotation rot = KDL::Rotation().Quaternion(pose[3], pose[4], pose[5], pose[6]);
  rot.DoRotX(3.14);
  rot.GetQuaternion(pose[3], pose[4], pose[5], pose[6]);

  std::vector<double> dim = table_monitor_->get_object_dimensions(object_id);
  
  // half the height of the object + length of gripper + hover height 
  pose[2] += (dim[2]/2.0) + (0.135) + hover_height;
}


void MotionCoordinator::stop(std::string planning_component)
{
  stop_motion(planning_component);
  sleep(replan_delay_);
  allow_motion(planning_component);
}

} // namespace motion_coordinator