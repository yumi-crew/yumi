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

#include <abb_egm_hardware/abb_egm_hardware_sim.hpp>


void spin(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exe)
{
  exe->spin();
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto robot = std::make_shared<abb_egm_hardware::AbbEgmHardwareSim>("abb_egm_hardware");

  // initialize the robot
  if (robot->init() != hardware_interface::HW_RET_OK)
  {
    fprintf(stderr, "Failed to initialize hardware");
    return -1;
  }


  // // Now load and initialize the controllers
  // As there is no ROS2 equivalent to ROS1 nodegroups we will manually pass along namespace
  std::string nodegroup_namespace = argv[2];
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  controller_manager::ControllerManager controller_manager(robot, executor, nodegroup_namespace + "/controller_manager"); 

  controller_manager.load_controller("ros_controllers", "ros_controllers::JointPositionController",
                                     "joint_position_controller");
  
  // Pass namespace to controllers as well
  auto controllers = controller_manager.get_loaded_controller();
  for(auto c : controllers)
  {
    auto l_node = c->get_lifecycle_node();
    l_node->declare_parameter("namespace", nodegroup_namespace);
  }


  // there is no async spinner in ROS 2, so we have to put the spin() in its own thread
  auto future_handle = std::async(std::launch::async, spin, executor);

  // we can either configure each controller individually through its services
  // or we use the controller manager to configure every loaded controller
  if (controller_manager.configure() != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS)
  {
    RCLCPP_ERROR(controller_manager.get_logger(), "at least one controller failed to configure");
    return -1;
  }
  // and activate all controller
  if (controller_manager.activate() != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS)
  {
    RCLCPP_ERROR(controller_manager.get_logger(), "at least one controller failed to activate");
    return -1;
  }

  rclcpp::Rate rate(250.0);

  hardware_interface::hardware_interface_ret_t ret;
  while (rclcpp::ok())
  {
    ret = robot->read();

    if (ret != hardware_interface::HW_RET_OK)
    {
      fprintf(stderr, "read failed!\n");
    }

    controller_manager.update();

    ret = robot->write();
    if (ret != hardware_interface::HW_RET_OK)
    {
      fprintf(stderr, "write failed!\n");
    }

    rate.sleep();
  }

  executor->cancel();

  fprintf(stderr, "Cancelled");

  return 0;
}