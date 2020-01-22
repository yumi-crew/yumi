
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>
#include <cmath>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include <ros2_control_interfaces/msg/joint_control.hpp>

#include <angles/angles.h>

#include <rws_clients/robot_manager_client.hpp>

#include <abb_robot_manager_interfaces/srv/is_ready.hpp>
#include <abb_robot_manager_interfaces/srv/stop_egm.hpp>
#include <abb_robot_manager_interfaces/srv/start_egm.hpp>



/*    TESTING TOOLS - SINE WAVE
 * 
 *    Streams two periods of a sinewave which starts out at joint position = initial joint position as commands
 *    to a selected joint. 
 * 
 *    Effectively asking the joint to track a sine wave reference for two periods.
 */



std::array<std::string, 7> joint_names;
std::string node_name_rmc = "robot_manager_client";
auto robot_manager_client = std::make_shared<rws_clients::RobotManagerClient>(node_name_rmc);


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

std::vector<double> recieved_joint_state_;

double a = 20;   // amplitude
double f = 0.5;  // frequency (sine wave)
double p = 0;    // phase

rclcpp::Clock::SharedPtr klokke = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
auto start = klokke->now().seconds();

int joint_nr;

void callback(sensor_msgs::msg::JointState::UniquePtr msg)
{
  printf("callback called\n");
  for(int i = 0; i < 7; i++)
  {
    recieved_joint_state_.push_back(msg->position[i]);
  }
}


void generate_new_command(ros2_control_interfaces::msg::JointControl *cmd_msg)
{
  auto now = klokke->now().seconds();
  auto t = now - start;
  
  for(int i = 0; i < 7; ++i)
  {
    cmd_msg->joints.push_back(joint_names[i]);
    cmd_msg->goals.push_back( recieved_joint_state_[i]);
  }
  cmd_msg->header.stamp = rclcpp::Clock().now();
  cmd_msg->goals[joint_nr] += angles::from_degrees(a*std::sin(2*3.14*f*t + p));
  printf("degrees additon = %f\n", (a*std::sin(2*3.14*f*t + p)));
     
}



int main(int argc, char * argv[])
{
  // Ctrl+C handler
  signal(SIGINT, signal_callback_handler);


  rclcpp::init(argc, argv);

  // making node
  auto node = rclcpp::Node::make_shared("sine_wave_inf");
  std::string nodegroup_namespace = node->get_namespace();
  joint_nr = std::stoi(argv[1])-1;



  // returns 0 if equal
  if( nodegroup_namespace.compare("/r") == 0 )
  {
    joint_names = { 
    "yumi_joint_1_r", "yumi_joint_2_r", "yumi_joint_7_r", "yumi_joint_3_r",
    "yumi_joint_4_r", "yumi_joint_5_r", "yumi_joint_6_r"
    };
  }
  else
  {
    joint_names = { 
      "yumi_joint_1_l", "yumi_joint_2_l", "yumi_joint_7_l", "yumi_joint_3_l",
      "yumi_joint_4_l", "yumi_joint_5_l", "yumi_joint_6_l"
    };
  }

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
  

  // Start EGM mode-----------------------------------------------------------------------------------------------------
  if(!robot_manager_client->start_egm())
  {
    printf("Failed to start egm \n");
  }
  //--------------------------------------------------------------------------------------------------------------------


  usleep(2*1000000);
  

  // creating subscription
  auto joint_state_subscription = node->create_subscription<sensor_msgs::msg::JointState>(nodegroup_namespace+"/joint_states", 10, callback);
  // creating publisher
  auto joint_command_publisher = node->create_publisher<ros2_control_interfaces::msg::JointControl>(nodegroup_namespace+"/joint_commands", 10);



  // publishing rate at 250 hz.
  rclcpp::WallRate loop_rate(250);
  while (rclcpp::ok()) 
  {
      
    // if recieved a state from robot, update reference.
    if(recieved_joint_state_.size() > 7)
    {
      ros2_control_interfaces::msg::JointControl msg;
      generate_new_command(&msg);
      joint_command_publisher->publish(msg);
    }
    else{
      printf("waiting for incoming state\n");
    }   


    rclcpp::spin_some(node);
    loop_rate.sleep();
  }



  printf("motion completed, please ctrl+c\n");
  while(1)
  {
    usleep(1*1000000);
  }

  rclcpp::shutdown();
  return 0;
}
