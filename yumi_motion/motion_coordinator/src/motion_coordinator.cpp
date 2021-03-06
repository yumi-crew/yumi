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

  yumi_manager_ = std::make_shared<rws_clients::RobotManagerClient>(node_);
  left_gripper_ = std::make_shared<rws_clients::GripperClient>(node_, "/l");
  right_gripper_ = std::make_shared<rws_clients::GripperClient>(node_, "/r");
  moveit2_wrapper_ = std::make_shared<moveit2_wrapper::Moveit2Wrapper>(node_);
  joint_state_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>("joint_states", 
    10, std::bind(&MotionCoordinator::joint_state_callback, this, std::placeholders::_1));

  
  rclcpp::Parameter param = node_->get_parameter("robot_description_path");
  std::string path = param.as_string();
  urdf::Model robot_model;
  if(!robot_model.initFile(path))
  {
    std::cout << "[ERROR] Unable to load urdf" << std::endl;
    return false;
  }
  kdl_wrapper_ = std::make_shared<KdlWrapper>(robot_model);
  if(!kdl_wrapper_->init())
  {
    std::cout << "[ERROR] kdl wrapper failed to initialize." << std::endl;
    return false;
  }
  if(!yumi_manager_->init())
  {
    std::cout << "[ERROR] yumi_manager failed to initialize." << std::endl;
    return false;
  }
  if(!left_gripper_->init())
  {
    std::cout << "[ERROR] left_gripper failed to initialize." << std::endl;
    return false;
  }
  if(!right_gripper_->init())
  {
    std::cout << "[ERROR] right_gripper failed to initialize." << std::endl;
    return false;
  }
  if(!moveit2_wrapper_->init())
  {
    std::cout << "[ERROR] moveit2_wrapper failed to initialize." << std::endl;
    return false;
  }

  object_manager_ = std::make_shared<moveit2_wrapper::ObjectManager>(moveit2_wrapper_->get_planning_interface());
  if(!object_manager_->init())
  {
    std::cout << "[ERROR] object_manager failed to initialize." << std::endl;
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
  
  if(!object_manager_->activate())
  {
    std::cout << "[ERROR] object_manager failed to activate." << std::endl;
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
  std::cout << "move_to_pose() called for planning component '" << planning_component << "'." << std::endl;

  if(replan && !blocking) 
  { 
    std::cout << "[ERROR] Replanning is only available for blocking motion." << std::endl;
    return;
  }
  auto planning_component_hash = moveit2_wrapper_->get_planning_components_hash();
  std::string ee_link = planning_component_hash->at(planning_component).ee_link;

  // If allready at pose, don't do anything.
  if(moveit2_wrapper_->pose_reached(planning_component, ee_link, pose, eulerzyx))
  {
    std::cout << "planning component '" << planning_component << "' is already at goal pose." << std::endl;
    return;
  }
   
  // If it should replan upon changed planning_scene
  if(replan)
  {
    // Run initial motion non-blocking.
    moveit2_wrapper_->pose_to_pose_motion(planning_component, ee_link, pose,eulerzyx, num_retries, visualize, replan, 
                                          false, speed_scale_, acc_scale_);

    while(!moveit2_wrapper_->pose_reached(planning_component, ee_link, pose, eulerzyx))
    {
      if(planning_component_hash->at(planning_component).should_replan)
      {
        stop(planning_component);
        should_replan_mutex_.lock();
        planning_component_hash->at(planning_component).should_replan = false; 
        should_replan_mutex_.unlock();
        moveit2_wrapper_->pose_to_pose_motion(planning_component, ee_link, pose, eulerzyx, num_retries, visualize, 
                                              replan, false, speed_scale_, acc_scale_);
      }
    }
    std::cout <<  "Goal pose of planning component '" << planning_component << "' considered reached." << std::endl;
    planning_component_hash->at(planning_component).in_motion = false;
  }
  else
  {
    moveit2_wrapper_->pose_to_pose_motion(planning_component, ee_link, pose, eulerzyx, num_retries, visualize, replan, 
                                          blocking, speed_scale_, acc_scale_);
  }
}


void MotionCoordinator::move_to_object(std::string planning_component, std::string object_id, int num_retries,   
                                       double hover_height, bool visualize, bool blocking, bool replan)
{
  std::cout << "move_to_object() called for planning component '"<< planning_component << "'." << std::endl;

  if(replan && !blocking) 
  { 
    std::cout <<  "[ERROR] Replanning is only available for blocking motion." << std::endl;
    return;
  }

  auto planning_components_hash = moveit2_wrapper_->get_planning_components_hash();
  std::string ee_link = planning_components_hash->at(planning_component).ee_link;

  std::vector<double> pose = object_manager_->find_object(object_id);

  // If object cant be found.
  if(pose.empty())
  { 
    stop(planning_component);
    std::cout <<  "[ERROR] Can't find object." << std::endl;
    return; 
  }
  else pose[2] += hover_height;

  // If already at object, don't do anything.
  if(moveit2_wrapper_->pose_reached(planning_component, ee_link, pose, false))
  {
    std::cout << "planning component '" << planning_component << "' is already at goal pose." << std::endl;
    return;
  }

  // If it should replan upon changed planning_scene.
  if(replan)
  {
    // Run initial motion non-blocking.
    moveit2_wrapper_->pose_to_pose_motion(planning_component, ee_link, pose, false, num_retries, visualize, replan, 
                                          false, speed_scale_, acc_scale_);

    while(!moveit2_wrapper_->pose_reached(planning_component, ee_link, pose, false))
    {
      if(planning_components_hash->at(planning_component).should_replan)
      {
        stop(planning_component);
        pose = object_manager_->find_object(object_id); 

        if(pose.empty())
        { 
          std::cout <<  "[ERROR] Can't find object." << std::endl;
          continue; 
        }
        else pose[2] += hover_height;

        should_replan_mutex_.lock();
        planning_components_hash->at(planning_component).should_replan = false; 
        should_replan_mutex_.unlock();
        moveit2_wrapper_->pose_to_pose_motion(planning_component, ee_link, pose, false, num_retries, visualize, replan, 
                                              false, speed_scale_, acc_scale_);
        
      }
    }
    std::cout <<  "Goal pose of planning component '" << planning_component << "' considered reached." << std::endl;
    planning_components_hash->at(planning_component).in_motion = false;
  }
  else
  {
    moveit2_wrapper_->pose_to_pose_motion(planning_component, ee_link, pose, false, num_retries, visualize, replan, 
                                          blocking, speed_scale_, acc_scale_);
  }
}


bool MotionCoordinator::linear_move_to_pose(std::string planning_component, std::vector<double> pose, bool eulerzyx, 
                                            int num_retries, bool visualize, bool blocking, bool collision_checking, 
                                            double percentage, double speed_scaling, double acc_scaling)                                 
{
  std::cout <<  "linear_move_to_pose() called for planning component '" << planning_component  << "'." << std::endl;
  
  int retries_left = num_retries;
  auto planning_components_hash = moveit2_wrapper_->get_planning_components_hash();
  std::string ee_link = planning_components_hash->at(planning_component).ee_link;

  // If already at pose, don't do anything
  if(moveit2_wrapper_->pose_reached(planning_component, ee_link, pose, eulerzyx))
  {
    std::cout << "planning component '" << planning_component << "' is already at goal pose." << std::endl;
    return true;
  }

  // Verify the pose is a valid pose.
  // if(!moveit2_wrapper_->pose_valid(planning_component, ee_link, pose, eulerzyx))
  // {
  //   std::cout << "[ERROR] The given goal pose is not valid. Aborting." << std::endl;
  //   return false;
  // }

  bool retry = false;
  std::vector<double> new_pose(6);
  std::vector<double> org_pose = moveit2_wrapper_->find_pose(ee_link); // Given using quaternions.

  bool success = moveit2_wrapper_->cartesian_pose_to_pose_motion(planning_component, ee_link, pose, eulerzyx, visualize,
                                                                 false, blocking, collision_checking, percentage, 
                                                                 speed_scaling, acc_scaling);
  while(!success && retries_left)
  {
    std::cout << " ---- new attempt" << std::endl;
    --retries_left;
    
    auto state = equivalent_state(planning_component, org_pose, false);
    auto cur_state = moveit2_wrapper_->get_current_state(planning_component);
    double dif = 0;
    for(int i = 0; i < state.size(); i++){ dif += abs(state[i]-cur_state[i]); }
    if( (!state.empty()) && (dif > 0.25) && moveit2_wrapper_->state_to_state_motion(planning_component, state, 5, false, 
                                                                                  false, true) )
    {
      std::cout << " ---- Trying with equivalent_state" << std::endl;
      success = moveit2_wrapper_->cartesian_pose_to_pose_motion(planning_component, ee_link, pose, eulerzyx, visualize,
                                                                false, blocking, collision_checking, percentage, 
                                                                speed_scaling, acc_scaling);
    }
    else
    {
      std::cout << " --- fallback" << std::endl;

      // Find a random pose nearby.
      new_pose = get_random_nearby_pose(org_pose, 0.1, 20, 40);

      // Move to random nearby pose.
      moveit2_wrapper_->pose_to_pose_motion(planning_component, ee_link, new_pose, true, 3, false, false, true);

      // Find a another random pose nearby.
      new_pose = get_random_nearby_pose(org_pose, 0.1, 50, 150);

      // Move to antoher random nearby pose.
      moveit2_wrapper_->pose_to_pose_motion(planning_component, ee_link, new_pose, true, 3, false, false, true);
    
      // Move end-effector back to original pose.
      retry = moveit2_wrapper_->pose_to_pose_motion(planning_component, ee_link, org_pose, false, 3, false, false,true);                                                              

      // Try again.
      if(retry) success = moveit2_wrapper_->cartesian_pose_to_pose_motion(planning_component, ee_link, pose, eulerzyx,
                                                                          visualize, false, blocking,collision_checking, 
                                                                          percentage, speed_scaling, acc_scaling);  
    }

    if(success) break;
    
    if((!success) && (!retries_left))
    {
      std::cout << " ***  linear_move_to_pose() was not able to find a valid solution within " 
                << num_retries << " attempts.\n";
      return false;
    } 
  }

  if(moveit2_wrapper_->pose_reached(planning_component, ee_link, pose, eulerzyx)) return true;
  else
  {
    sleep(1);
    return moveit2_wrapper_->pose_reached(planning_component, ee_link, pose, eulerzyx);
  }
  
}


bool MotionCoordinator::linear_move_to_object(std::string planning_component, std::string object_id,int num_retries,
                                              double hover_height, bool visualize,bool blocking,bool collision_checking, 
                                              double percentage, double speed_scaling, double acc_scaling)
{
  std::cout <<  "linear_move_to_object() called for planning component '" << planning_component << "'." << std::endl;

  auto planning_components_hash = moveit2_wrapper_->get_planning_components_hash();
  std::string ee_link = planning_components_hash->at(planning_component).ee_link;

  std::vector<double> pose = object_manager_->find_object(object_id); 
  
  // If object cant be found.
  if(pose.empty())
  { 
    stop(planning_component);
    std::cout <<  "[ERROR] Can't find object." << std::endl;
    return false; 
  }
  else pose[2] += hover_height;

  // If already at goal pose, don't do anything
  if(moveit2_wrapper_->pose_reached(planning_component, ee_link, pose, false))
  {
    std::cout << "planning component '" << planning_component << "' is already at goal pose." << std::endl;
    return true;
  }

  return linear_move_to_pose(planning_component, pose, false, num_retries, visualize, blocking, 
                             collision_checking, percentage, speed_scaling, acc_scaling);
}


int MotionCoordinator::pick_object(std::string planning_component, std::string object_id,   
                                   std::vector<std::string> allowed_collisions, int num_retries, double hover_height, 
                                   bool visualize)
{
  std::cout << "pick_object() called for planning component '" << planning_component << "'." << std::endl;

  // Gripper links
  std::vector<std::string> gripper_links = {"gripper_r_finger_l", "gripper_r_finger_r"};

  // Get end effector link
  auto planning_components_hash = moveit2_wrapper_->get_planning_components_hash();
  std::string ee_link = planning_components_hash->at(planning_component).ee_link;

  std::vector<double> grip_pose = object_manager_->find_object(object_id); // quaternions
  std::vector<double> object_dimensions = object_manager_->get_object_dimensions(object_id);

  // If object cant be found.
  if(grip_pose.empty() || object_dimensions.empty())
  { 
    stop(planning_component);
    std::cout <<  "[ERROR] Can't find object." << std::endl;
    return error::PLANNING_SCENE_FAIL; 
  }

  std::vector<double> hover_pose = grip_pose; hover_pose[2] += hover_height;

  // Disable collision for the object ot be picked
  moveit2_wrapper_->disable_collision(object_id, true);

  // Move to hover pose and open gripper enough to pick.
  move_to_pose(planning_component, hover_pose, false, num_retries, visualize, true, false);
  jog_gripper(planning_component, object_dimensions[0]+grip_margin_, true);

  // Allow collision between fingers and bin
  for(auto link : gripper_links){ moveit2_wrapper_->disable_collision("bin6", true, link); }
  
  // Verify the grip pose is valid, give up if not.
  if(!moveit2_wrapper_->pose_valid(planning_component, ee_link, grip_pose, false))
  {
    std::cout << "[ERROR] The given goal pose is not valid. Aborting." << std::endl;
    close_gripper(planning_component, true);
    object_manager_->remove_object_from_scene(object_id, false);
    return error::INVALID_POSE;
  }

  // Linear move to gripping pose, give up if unable.
  if(!linear_move_to_pose(planning_component, grip_pose, false, num_retries, visualize, true, true, 1.0, 0.3, 0.3))
  {
    std::cout << "[ERROR] Unable to find a linear path. Aborting." << std::endl;
    close_gripper(planning_component, true);
    object_manager_->remove_object_from_scene(object_id, false);
    return error::LINEAR_PLAN_FAIL;
  }

  // Grip object
  object_manager_->attach_object(object_id, ee_link);
  
  // Disable collision between pick object and elements in the enviroment 
  for(auto object : allowed_collisions) 
  { 
    moveit2_wrapper_->disable_collision(object_id, true, object); 
  }
  grip_in(planning_component, true);
  

  // Linear move back to hover point. If linear motion is not possible, try ordinary motion.
  if(!linear_move_to_pose(planning_component, hover_pose, false, 0, visualize, true, true, 1.0, 1.0, 1.0))
  {
    move_to_pose(planning_component, hover_pose, false, num_retries, visualize, true, false);
  }

  // Re-enable collision checking between pick object and elements in the enviroment .
  for(auto object : allowed_collisions) 
  { 
    moveit2_wrapper_->disable_collision(object_id, false, object); 
  }

  // Re-enable collision checking between fingers and bin.
  for(auto link : gripper_links){ moveit2_wrapper_->disable_collision("bin6", false, link); }

  // Check if yumi's gripper contain the object, return if not.
  if(moveit2_wrapper_->gripper_closed(planning_component))
  {
    std::cout << "[ERROR] Pick failed. Aborting." << std::endl;
    object_manager_->detatch_object(object_id);
    return error::GRIP_FAIL;
  }

  return 0;
}


int MotionCoordinator::place_in_object(std::string planning_component, std::string object_id, int num_retries,  
                                         double hover_height, bool visualize)
{
  std::cout << "place_object() called for planning component '" << planning_component << "'." << std::endl;

  // Get end effector link
  auto planning_components_hash = moveit2_wrapper_->get_planning_components_hash();
  std::string ee_link = planning_components_hash->at(planning_component).ee_link;

  // Object to be placed
  std::string object = object_manager_->object_held(ee_link);

  // Check if yumi's gripper contain an object, return if not.
  if(moveit2_wrapper_->gripper_closed(planning_component))
  {
    std::cout << "[ERROR] YuMi have lost the object to be placed. Aborting." << std::endl;
    object_manager_->detatch_object(object);
    return error::GRIP_FAIL;
  }

  // linear move to hover point
  if(!linear_move_to_object(planning_component, object_id, 0, hover_height, visualize, true, true, 1.0))
  {
    // If linear motion is not possible, try ordinary motion.
    move_to_object(planning_component, object_id, num_retries, hover_height, visualize, true, false);
  } 

  // Check if yumi has dropped the object
  if(moveit2_wrapper_->gripper_closed(planning_component))
  {
    std::cout << "[ERROR] YuMi has lost the object" << std::endl;
    object_manager_->detatch_object(object);
    return error::GRIP_FAIL;
  }

  // linear move down to drop point. If drop point cant be reached, drop object at current pose.
  if(!linear_move_to_object(planning_component, object_id, 0, hover_height-0.05, visualize, true, true, 1.0))
  {
    open_gripper(planning_component, true);
    object_manager_->detatch_object(object);
    close_gripper(planning_component, true);
    return 0;
  }
 
  // Let go object
  open_gripper(planning_component, true);
  object_manager_->detatch_object(object);
  

  // Linear move back to hover point. If linear motion is not possible, try ordinary motion.
  if(!linear_move_to_object(planning_component, object_id, 0, hover_height, visualize, true, true, 1.0))
  {
    move_to_object(planning_component, object_id, num_retries, hover_height, visualize, true, false);
  }

  close_gripper(planning_component, true);
  return 0;
}


void MotionCoordinator::move_to_state(std::string planning_component, std::vector<double> state, int num_retries, 
                                      bool visualize, bool blocking, bool replan)
{
  std::cout << "move_to_state() called for planning component '" << planning_component << "'." << std::endl;

  if(replan && !blocking) 
  { 
    std::cout << "[ERROR] Replanning is only available for blocking motion." << std::endl;
    return;
  }

  auto planning_component_hash = moveit2_wrapper_->get_planning_components_hash();

  // If allready at state, don't do anything.
  if(moveit2_wrapper_->state_reached(planning_component, state))
  {
    std::cout << "planning component '" << planning_component << "' is already at goal state." << std::endl;
    return;
  }

  // If it should replan upon changed planning_scene
  if(replan)
  {
    // Run initial motion non-blocking.
    moveit2_wrapper_->state_to_state_motion(planning_component, state, num_retries, visualize, replan, false,
                                            speed_scale_, acc_scale_); 
                                            
    while(!moveit2_wrapper_->state_reached(planning_component, state))
    {
      if(planning_component_hash->at(planning_component).should_replan)
      {
        stop(planning_component);
        moveit2_wrapper_->state_to_state_motion(planning_component, state, num_retries, visualize, replan, false, 
                                                speed_scale_, acc_scale_); 
                                                
        should_replan_mutex_.lock();
        planning_component_hash->at(planning_component).should_replan = false; 
        should_replan_mutex_.unlock();
      }
    }
    std::cout <<  "Goal state of planning component '" << planning_component << "' considered reached." << std::endl;
    planning_component_hash->at(planning_component).in_motion = false;
  }
  else
  {
    moveit2_wrapper_->state_to_state_motion(planning_component, state, num_retries, visualize, false, blocking, 
                                            speed_scale_, acc_scale_);                                    
  }
}


void MotionCoordinator::move_to_home(std::string planning_component, int num_retries, bool visualize, bool blocking, 
                                     bool replan)
{
  std::cout << "move_to_home() called for planning component '" << planning_component << "'." << std::endl;

  if(replan && !blocking) 
  { 
    std::cout << "[ERROR] Replanning is only available for blocking motion." << std::endl;
    return;
  }

  auto planning_component_hash = moveit2_wrapper_->get_planning_components_hash();
  std::vector<double> home =  planning_component_hash->at(planning_component).home_configuration;
  move_to_state(planning_component, home, num_retries, visualize, blocking, replan);
}


std::vector<double> MotionCoordinator::random_move_object(std::string object_id, double side_shift)
{ 
  std::mt19937 rng((unsigned)time(NULL));
  std::uniform_int_distribution<int> gen(0, 1000);
  int r = gen(rng);
  double shift = 0;
  if(r%2) shift = side_shift;
  else shift = -side_shift;
  
  std::vector<double> pose = object_manager_->find_object(object_id);
  pose[1] += shift; 
  
  object_manager_->move_object(object_id, pose);

  auto planning_components_hash = moveit2_wrapper_->get_planning_components_hash();
  should_replan_mutex_.lock();
  planning_components_hash->at("left_arm").should_replan = true;
  planning_components_hash->at("right_arm").should_replan = true;
  planning_components_hash->at("both_arms").should_replan = true;
  should_replan_mutex_.unlock();
  return {pose[0], pose[1], pose[2]};
}


std::vector<double> MotionCoordinator::random_nearby_position(std::vector<double> old_position, double side_shift)
{ 
  std::mt19937 rng((unsigned)time(NULL));
  std::uniform_int_distribution<int> gen(0, 1000);
  int r = gen(rng);
  double shift = 0;
  if(r%2) shift = side_shift;
  else shift = -side_shift;

  std::mt19937 rng2((unsigned)time(NULL));
  std::uniform_int_distribution<int> gen2(0, 1);
  int index = gen2(rng2);

  old_position[index] += shift; 
  return {old_position[0], old_position[1], old_position[2]};
}


std::vector<double> MotionCoordinator::random_orientation(double l_limit, double u_limit)
{ 
  KDL::Rotation rot = KDL::Rotation::Identity();

  // Find random axis
  std::mt19937 rng_axis((unsigned)time(NULL));
  std::uniform_int_distribution<int> gen_axis(0, 1);
  int axis = gen_axis(rng_axis);

  // Find random angle
  std::mt19937 rng_angle((unsigned)time(NULL));
  std::uniform_int_distribution<int> gen_angle(l_limit, u_limit);
  int angle = gen_angle(rng_angle);
  
  // Apply random rotation
  if(axis == 0) { rot.DoRotZ(angles::from_degrees(angle)); }
  else if(axis == 1) { rot.DoRotY(angles::from_degrees(angle)); }

  rot.DoRotX(angles::from_degrees(180));
  std::vector<double> zyx(3);
  rot.GetEulerZYX(zyx[0], zyx[1], zyx[2]);
  return {angles::to_degrees(zyx[0]), angles::to_degrees(zyx[1]), 180};
}


std::vector<double> MotionCoordinator::get_random_nearby_pose(std::vector<double> org_pose, double side_shift, 
                                                           double l_limit,  double u_limit)
{  
  // Find random new nearby position.
  std::vector<double> new_position = random_nearby_position({org_pose[0], org_pose[1], org_pose[2]}, side_shift);
  std::vector<double> new_orien = random_orientation(l_limit, u_limit);

  std::vector<double> new_pose(6);
  new_pose[0] = new_position[0];
  new_pose[1] = new_position[1];
  new_pose[2] = new_position[2];
  new_pose[3] = new_orien[0];
  new_pose[4] = new_orien[1];
  new_pose[5] = new_orien[2];
  return new_pose;
}


void MotionCoordinator::remove_object(std::string object_id)
{ 
  object_manager_->remove_object_from_scene(object_id, true); 

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


void MotionCoordinator::stop(std::string planning_component)
{
  stop_motion(planning_component);
  sleep(replan_delay_);
  allow_motion(planning_component);
}


void MotionCoordinator::add_object(std::string object_id, std::vector<double> pose, bool eulerzyx, std::vector<float> rgba)
{
  if(eulerzyx)
  {
    geometry_msgs::msg::PoseStamped msg = moveit2_wrapper_->pose_vec_to_msg(pose, eulerzyx);
    pose[3] = msg.pose.orientation.x;
    pose[4] = msg.pose.orientation.y;
    pose[5] = msg.pose.orientation.z;
    pose.push_back(msg.pose.orientation.w);
  }
  if(!rgba.empty())
  {
    object_manager_->add_object_to_scene(object_id, pose, false); 
    object_manager_->set_object_color(object_id, rgba);
  }
  else object_manager_->add_object_to_scene(object_id, pose, true); 
}


void MotionCoordinator::grip_in(std::string planning_component, bool blocking)
{
  if(planning_component == "left_arm") left_gripper_->grip_in();
  if(planning_component == "right_arm") right_gripper_->grip_in();

  if(blocking){ sleep(1.5); } // Takes at most approx 1.2 seconds to close
}


void MotionCoordinator::grip_out(std::string planning_component, bool blocking)
{
  if(planning_component == "left_arm") left_gripper_->grip_out();
  if(planning_component == "right_arm") right_gripper_->grip_out();

  if(blocking){ sleep(1.5); } // Takes at most approx 1.2 seconds to close
}


std::vector<double> MotionCoordinator::equivalent_state(std::string planning_component, std::vector<double> pose, 
                                                        bool eulerzyx)
{
  KDL::Vector vec(pose[0], pose[1], pose[2]);
  KDL::Rotation rot = KDL::Rotation::Identity();

  if(eulerzyx)
  {
    rot = KDL::Rotation::EulerZYX(pose[3], pose[4], pose[5]);
  }
  else
  {
    rot = KDL::Rotation::Quaternion(pose[3], pose[4], pose[5], pose[6]);
  }

  std::vector<float> q_seed_left_l = {-2.83, -1.18, 2.64, -0.47, 1.89, 0.96, 1.56};
  std::vector<float> q_seed_right_l = {-1.95, -1.95, 0.94, 0.87, -3.55, 2.87, 2.41}; 
  std::vector<float> q_seed_left_r = {-0.68, -0.067, 0.0054, 0.17, -0.63, 0.53, 0.23};
  std::vector<float> q_seed_middle_1r = {1.73, 0.34, -2.16, 0.56, -3.9, -0.35, 2.48};
  std::vector<float> q_seed_middle_2r = {-1.81,-0.5, 1.27, 0.23, -0.04, 0.52, 2.41};
  std::vector<float> q_seed_middle_3r = {-1.61, -1.68, 0.97, 0.91, 2.35, 2.07, 2.21};
  std::vector<float> q_seed_middle_4r ={-2.64, -1.7, 0.6, 0.75, 1.82, 1.69, 1.29};
  std::vector<float> q_seed_middle_l = {-2.40, -0.3177, 2.62, 0.46, 1.86, 0.69, 1.39};
  std::vector<float> q_seed_middle_2l = {-1.68, -0.88, 1.57, 0.60, 2.10, 1.34, 1.66};
  std::vector<float> q_seed_1 = {0.24, -0.69, -0.77, 0.35, 2.2, -0.021, -1};
  std::vector<float> q_seed_2 = { 0, 0, 0, 0.2, 0, 0, 0 };
  std::vector<float> q_seed_3 = {0.9, -2, -1.7, 0.76, -2.7, 1,1};
  std::vector<float> q_seed_4 = {-0.78, -1.8, 1.5, 0.12, 1.15, 0.85, 0.92};
  std::vector<float> q_seed_6 = {-0.97, -1.8, 1.4, 0.34, 0.61, 1.3, 0.76};
  std::vector<float> q_seed_7 = {1.15, -1.5, -1.74, -0.64, -0.61, 0.54, -0.89};
  std::vector<double> q_seed_cur = moveit2_wrapper_->get_current_state(planning_component);

  // Add or subtract
  std::mt19937 rng_even((unsigned)time(NULL));
  std::uniform_int_distribution<int> gen_even(0, 1000);

  // Random seed 
  std::mt19937 rng((unsigned)time(NULL));
  std::uniform_int_distribution<int> gen(100, 200);
  
  std::vector<double> q_seed_rand = q_seed_cur;
  for(int i = 0; i < q_seed_cur.size(); i++)
  {
    int r = gen(rng);
    int num = gen_even(rng_even);
    if(num%2) q_seed_rand[i] += r/100;
    else q_seed_rand[i] -= r/100;
  }

  std::vector<KDL::JntArray> seeds = 
  { kdl_wrapper_->stdvec_to_jntarray(q_seed_middle_1r),
    kdl_wrapper_->stdvec_to_jntarray(q_seed_middle_2r),
    kdl_wrapper_->stdvec_to_jntarray(q_seed_middle_3r),
    kdl_wrapper_->stdvec_to_jntarray(q_seed_middle_4r),
    kdl_wrapper_->stdvec_to_jntarray(q_seed_left_r),
    // kdl_wrapper_->stdvec_to_jntarray(q_seed_left_l),
    // kdl_wrapper_->stdvec_to_jntarray(q_seed_right_l),
    // kdl_wrapper_->stdvec_to_jntarray(q_seed_middle_l),
    // kdl_wrapper_->stdvec_to_jntarray(q_seed_middle_2l),
    // kdl_wrapper_->stdvec_to_jntarray(q_seed_1),
    // kdl_wrapper_->stdvec_to_jntarray(q_seed_2),
    // kdl_wrapper_->stdvec_to_jntarray(q_seed_3),
    // kdl_wrapper_->stdvec_to_jntarray(q_seed_4),
    kdl_wrapper_->stdvec_to_jntarray(q_seed_rand),
    // kdl_wrapper_->stdvec_to_jntarray(q_seed_6),
    // kdl_wrapper_->stdvec_to_jntarray(q_seed_7),
    kdl_wrapper_->stdvec_to_jntarray(q_seed_cur) };

  KDL::JntArray jntarr(7);
  KDL::Frame frame(rot, vec);
  
  for(auto seed : seeds)
  {
    // IK
    if(planning_component == "left_arm") { jntarr = kdl_wrapper_->inverse_kinematics_left(frame, seed); }
    else{ jntarr = kdl_wrapper_->inverse_kinematics_right(frame, seed); }   
    
    // Convert to std::vector
    std::vector<double> state;
    for(int i = 0; i < jntarr.rows(); ++i)
    { 
      if(jntarr(i) != 0) state.push_back(jntarr(i)); 
    }
    // Find diff
    double dif = 0; 
    for(int i = 0; i < state.size(); i++) 
    { 
      dif += abs(q_seed_4[i]-state[i]);
    }
    // If sufficiently different and not empty.
    if( (dif > 0.5) &&  (jntarr.data.sum() > 0)) return state;
  }
  
  return {};
}


void MotionCoordinator::print_matrix(Eigen::Matrix4d mat)
{
  std::cout << mat(0,0) << " " << mat(0,1) << " " << mat(0,2) << " " << mat(0,3) << std::endl;
  std::cout << mat(1,0) << " " << mat(1,1) << " " << mat(1,2) << " " << mat(1,3) << std::endl;
  std::cout << mat(2,0) << " " << mat(2,1) << " " << mat(2,2) << " " << mat(2,3) << std::endl;
  std::cout << mat(3,0) << " " << mat(3,1) << " " << mat(3,2) << " " << mat(3,3) << std::endl;
}


void MotionCoordinator::move_object(std::string object_id, std::vector<double> pose)
{ 
  double error = 1;
  object_manager_->move_object(object_id, pose); 
  std::vector<double> actual_pose(7);
  double allowed_deviation = 0.00005;
  
  while(error > allowed_deviation)
  {
    error = 0;
    actual_pose = object_manager_->find_object(object_id);
    for(int i = 0; i < actual_pose.size(); i++)
    {
      error += abs(actual_pose[i]-pose[i]);
    }

    if(error <= allowed_deviation) break;
    else object_manager_->move_object(object_id, pose); 
  }
}


void MotionCoordinator::jog_gripper(std::string planning_component, double goal_pos, bool blocking)
{
  std_msgs::msg::Float32 msg;
  msg.data = goal_pos/2.0; // gripper = joint + mimic_joint
  if(planning_component == "right_arm") right_gripper_->jog_gripper(goal_pos);
  else if(planning_component == "left_arm") left_gripper_->jog_gripper(goal_pos);

  double error = 0.0;
  double curr_pos = moveit2_wrapper_->gripper_pos(planning_component);
  if(blocking)
  {
    while( abs(curr_pos-goal_pos) < 0.001)
    {
      sleep(0.1);
      curr_pos = moveit2_wrapper_->gripper_pos(planning_component);
    }
  }
}

void MotionCoordinator::open_gripper(std::string planning_component, bool blocking)
{
  jog_gripper(planning_component, 0.025, blocking);
}


void MotionCoordinator::close_gripper(std::string planning_component, bool blocking)
{
  jog_gripper(planning_component, 0.0, blocking);
}


void MotionCoordinator::drop_object(std::string object_id, std::string planning_component)
{
  grip_out(planning_component, true);
  object_manager_->detatch_object(object_id);
}


void MotionCoordinator::grab_object(std::string object_id, std::string planning_component)
{
  grip_in(planning_component, true);

  // Get end effector link
  auto planning_components_hash = moveit2_wrapper_->get_planning_components_hash();
  std::string ee_link = planning_components_hash->at(planning_component).ee_link;
  object_manager_->attach_object(object_id, ee_link);
  moveit2_wrapper_->disable_collision(object_id, true, "gripper_l_finger_r");
  moveit2_wrapper_->disable_collision(object_id, true, "gripper_l_finger_l");
}

} // namespace motion_coordinator
