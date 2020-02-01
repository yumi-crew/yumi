#include <unistd.h>
#include <iostream>
#include <cstdlib>
#include <signal.h>
#include <string>

#include <rws_clients/robot_manager_client.hpp>

std::string node_name = "robot_manager_client";
auto robot_manager_client = std::make_shared<rws_clients::RobotManagerClient>(node_name);
    

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


int main(int argc, char* argv[])
{

  // Ctrl+C handler
  signal(SIGINT, signal_callback_handler);

  rclcpp::init(argc, argv);
  
  
  // Initialize the state machine
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


  // Start egm
  if(!robot_manager_client->start_egm())
  {
    printf("Failed to start egm \n");
  }

  // Just for this node, want to have a little delay 
  usleep(2*1000000);


  // Stop egm
  if(!robot_manager_client->stop_egm())
  {
    printf("Failed to stop egm\n");
  }


  return 0;
}