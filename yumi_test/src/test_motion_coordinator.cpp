// Copyright 2020 Markus Bjønnes and Marius Nilsen.
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

std::shared_ptr<motion_coordinator::MotionCoordinator> yumi_motion_coordinator;

// Ctr+C handler
void signal_callback_handler(int signum)
{
  std::cout << "Caught signal " << signum << std::endl;
  // Terminate EGM session
  yumi_motion_coordinator->terminate_egm_session();
  // Terminate ros node
  rclcpp::shutdown();
  // Terminate program
  exit(signum);
}

void spin(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exe)
{
  exe->spin();
}

int main(int argc, char** argv)
{
  // Ctrl+C handler
  signal(SIGINT, signal_callback_handler);
  rclcpp::init(argc, argv);

  yumi_motion_coordinator = std::make_shared<motion_coordinator::MotionCoordinator>("motion_coordinator");
  if(!yumi_motion_coordinator->init()) 
  {
    std::cout << "ERROR: MotionCoordinator failed to Initialize." << std::endl;
    return -1;  
  }

  // Spin the moveit2 node countiously in another thread via an executor
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(yumi_motion_coordinator->get_node());
  auto future_handle = std::async(std::launch::async, spin, executor);
  
  if(!yumi_motion_coordinator->activate())
  {
    std::cout << "ERROR: MotionCoordinator failed to activate." << std::endl;
    return -1;  
  } 

  while(1)
  {
    yumi_motion_coordinator->move_to_pose("right_arm", {0.4, 0.1, 0.3, 0, 0, 180}, true, 3, false, true, false);
    sleep(1);
    yumi_motion_coordinator->move_to_home("right_arm", 3);
  }

  std::cout << "Motion completed, please ctrl+c" << std::endl;
  while(1)
  {
    sleep(1);
  }
  return 0;
}
