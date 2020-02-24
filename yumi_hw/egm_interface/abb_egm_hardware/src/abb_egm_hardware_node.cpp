// Copyright 2019 Norwegian University of Science and Technology.
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

#include <rclcpp/rclcpp.hpp>

#include "controller_manager/controller_manager.hpp"

#include "abb_egm_hardware/abb_egm_hardware.hpp"



void spin(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exe)
{
  exe->spin();
}

int main(int argc, char* argv[])
{
  
  rclcpp::init(argc, argv);
  auto robot = std::make_shared<abb_egm_hardware::AbbEgmHardware>("abb_egm_hardware");
  hardware_interface::hardware_interface_ret_t ret;

  // To avoid a race condition, wait to ensure all parameter servers are ready.
  rclcpp::sleep_for(std::chrono::milliseconds(2000));


  // Initialize the robot
  if (robot->init() != hardware_interface::HW_RET_OK)
  {
    fprintf(stderr, "Failed to initialize hardware");
    return -1;
  }

  // Reads to initialize the joint arrays
  ret = robot->read();
  if (ret != hardware_interface::HW_RET_OK)
  {
    fprintf(stderr, "read failed!\n");
  }


  // Now load and initialize the controllers
  // As there is no ROS2 equivalent to ROS1 nodegroups we will manually pass along namespace
  std::string nodegroup_namespace = argv[1];
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  controller_manager::ControllerManager controller_manager(robot, executor, nodegroup_namespace+"/controller_manager"); 

  controller_manager.load_controller("controllers", "ros_controllers::JointStateController",
                                     "joint_state_controller");
  controller_manager.load_controller("controllers", "ros_controllers::JointTrajectoryController",
                                     "joint_trajectory_controller");


  // Pass namespace to controllers as well
  auto controllers = controller_manager.get_loaded_controller();
  for(auto c : controllers)
  {
    auto l_node = c->get_lifecycle_node();
    l_node->declare_parameter("namespace", nodegroup_namespace);
  }

  // there is no async spinner in ROS 2, so we have to put the spin() in its own thread
  auto future_handle = std::async(std::launch::async, spin, executor);

  // Controller manager transitions the cotnrollers lifecycle node from Unconfigured to Inactive state
  // by calling their respective on_configured() functions.
  if (controller_manager.configure() != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS)
  {
    RCLCPP_ERROR(controller_manager.get_logger(), "at least one controller failed to configure"); 
    return -1;
  }

  // Controller manager transitions the controllers lifecycle nodes from Inactive to Active state.
  // by running their respective on_activate() funcitons.
  if (controller_manager.activate() != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS)
  {
    RCLCPP_ERROR(controller_manager.get_logger(), "at least one controller failed to activate");
    return -1;
  }
  
  std::cout << "!!! entering control loop" << std::endl;
  // Real-time control loop
  while (rclcpp::ok())
  {
    // Reads into joint_position_ and joint_velocity_
    ret = robot->read();
    if (ret != hardware_interface::HW_RET_OK)
    {
      fprintf(stderr, "read failed!\n");
    }

    // (joint_position_controller): Updates joint_position_command_ 
    controller_manager.update();

    // Writes the contents of joint_position_command_ to robot
    ret = robot->write();
    if (ret != hardware_interface::HW_RET_OK)
    {
      fprintf(stderr, "write failed!\n");
    }
    
  }

  // teardown
  executor->cancel();
  fprintf(stderr, "Cancelled");

  return 0;
}