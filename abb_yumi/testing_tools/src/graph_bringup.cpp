/*  DEMO - Both arms - Both grippers
 *
 *
 */

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>
#include <cmath>
#include <unistd.h>
#include <cstdlib>

#include <rclcpp/rclcpp.hpp>

#include <angles/angles.h>

#include <ros2_control_interfaces/msg/joint_control.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <rws_clients/grip_client.hpp>
#include <rws_clients/robot_manager_client.hpp>


#include <abb_robot_manager_interfaces/srv/is_ready.hpp>
#include <abb_robot_manager_interfaces/srv/stop_egm.hpp>
#include <abb_robot_manager_interfaces/srv/start_egm.hpp>
#include <sg_control_interfaces/action/grip.hpp>



// Globals--------------------------------------------------------------------------------------------------------------

std::vector<double> recieved_joint_state_l;
std::vector<double> recieved_joint_state_r;
//home (1,2,7,3,4,5,6) ->  0.0, -130.0, 0.0, 30.0, 0.0, 40.0, 0.0};
//skal sendes som 1,2,7,3,4,5,6


std::array<std::string, 7> joint_names_l = { 
      "yumi_joint_1_l", "yumi_joint_2_l", "yumi_joint_7_l", "yumi_joint_3_l",
      "yumi_joint_4_l", "yumi_joint_5_l", "yumi_joint_6_l"
};
std::array<std::string, 7> joint_names_r = { 
      "yumi_joint_1_r", "yumi_joint_2_r", "yumi_joint_7_r", "yumi_joint_3_r",
      "yumi_joint_4_r", "yumi_joint_5_r", "yumi_joint_6_r"
};
std::string node_name_rmc = "bringup";
auto robot_manager_client = std::make_shared<rws_clients::RobotManagerClient>(node_name_rmc);

//----------------------------------------------------------------------------------------------------------------------




void signal_callback_handler(int signum ) 
{
  std::cout << "Caught signal " << signum << std::endl;
  // Stop EGM 
  robot_manager_client->stop_egm();
  // Terminate ros node
  rclcpp::shutdown();
  // Terminate program
  exit(signum);
}




int main(int argc, char * argv[])
{
  // Ctrl+C handler
  signal(SIGINT, signal_callback_handler);

  // Setup
  rclcpp::init(argc, argv);


  // Initialize the robot manager client
  if(!robot_manager_client->init())
  {
    printf("Failed to initialize the robot manager client \n");
    return -1;
  }
  // Blocking loop
  while(!robot_manager_client->robot_is_ready())
  {
    usleep(0.001*100000);
  }
  // Start EGM mode
  if(!robot_manager_client->start_egm())
  {
    printf("Failed to start egm \n");
  }

 
  
  //--------------------------------------------------------------------------------------------------------------------

  printf("waiting, please ctrl+c\n");
  while(1)
  {
    usleep(1*1000000);
  }
  
  return 0;
}
