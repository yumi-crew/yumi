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
std::string node_name_rmc = "robot_manager_client";
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


void callback_l(sensor_msgs::msg::JointState::UniquePtr msg)
{
  //printf("callback called left\n");
  for(int i = 0; i < 7; i++)
  {
    recieved_joint_state_l.push_back(msg->position[i]);
  }
}

void callback_r(sensor_msgs::msg::JointState::UniquePtr msg)
{
  //printf("callback called right \n");
  for(int i = 0; i < 7; i++)
  {
    recieved_joint_state_r.push_back(msg->position[i]);
  }
}



void generate_command_l(ros2_control_interfaces::msg::JointControl *cmd_msg, std::vector<double> joint_goals)
{ 
  for(int i = 0; i < 7; i++)
  {
    cmd_msg->joints.push_back(joint_names_l[i]);
    cmd_msg->goals.push_back(angles::from_degrees(joint_goals[i]));
  }
  cmd_msg->header.stamp = rclcpp::Clock().now();
}

void generate_command_r(ros2_control_interfaces::msg::JointControl *cmd_msg, std::vector<double> joint_goals)
{
  for(int i = 0; i < 7; i++)
  {
    cmd_msg->joints.push_back(joint_names_r[i]);
    cmd_msg->goals.push_back(angles::from_degrees(joint_goals[i]));
  }
  cmd_msg->header.stamp = rclcpp::Clock().now();
}




int main(int argc, char * argv[])
{
  // Ctrl+C handler
  signal(SIGINT, signal_callback_handler);

  // Setup
  rclcpp::init(argc, argv);
  auto node_l = rclcpp::Node::make_shared("demo_l");
  auto node_r = rclcpp::Node::make_shared("demo_r");
  std::string ns_l = "/l";
  std::string ns_r = "/r";


  // Initialize the left gripper
  std::string node_name_gac_l = "grip_client_left";
  auto grip_action_client_l = std::make_shared<rws_clients::GripClient>(node_name_gac_l, ns_l);
  if(!grip_action_client_l->init())
  {
    printf("Unable to initalize the left gripper client");
  }
  // Initialize the right gripper
  std::string node_name_gac_r = "grip_client_right";
  auto grip_action_client_r = std::make_shared<rws_clients::GripClient>(node_name_gac_r, ns_r);
  if(!grip_action_client_r->init())
  {
    printf("Unable to initalize the right gripper client");
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
  // Start EGM mode
  if(!robot_manager_client->start_egm())
  {
    printf("Failed to start egm \n");
  }

 
  // create subscription left
  auto joint_state_subscription_l = node_l->create_subscription<sensor_msgs::msg::JointState>(
    ns_l+"/joint_states", 
    10, 
    callback_l
  );
  // create publisher left
  auto joint_command_publisher_l = node_l->create_publisher<ros2_control_interfaces::msg::JointControl>(
    ns_l+"/joint_commands", 
    10
  );
  // create subscription right
  auto joint_state_subscription_r = node_r->create_subscription<sensor_msgs::msg::JointState>(
    ns_r+"/joint_states", 
    10, 
    callback_r
  );
  // create publisher right
  auto joint_command_publisher_r = node_r->create_publisher<ros2_control_interfaces::msg::JointControl>(
    ns_r+"/joint_commands", 
    10
  );



  rclcpp::spin_some(node_l);
  rclcpp::spin_some(node_r);
  usleep(2*1000000);

  std::vector<double> goal_l;
  goal_l.resize(7);
  std::vector<double> goal_r;
  goal_r.resize(7);

  bool sent = false;
  rclcpp::WallRate loop_rate(250);



  // The hardcoded movements start here---------------------------------------------------------------------------------
  
  // Command should be sent as {1,2,7,3,4,5,6}


  // BOTH : Go to home pos----------------------------------------------------------------------------------------------
  goal_l = {0.0, -130.0, 135.0, 30.0, 0.0, 30.0, 0.0};
  goal_r = {0.0, -130.0, -135.0, 30.0, 0.0, 30.0, 0.0};
  ros2_control_interfaces::msg::JointControl msg1l;  
  ros2_control_interfaces::msg::JointControl msg1r;
  generate_command_l(&msg1l, goal_l);
  generate_command_r(&msg1r, goal_r);

  // Publish
  while (rclcpp::ok()) 
  {
    // if recieved a state from robot, update reference.
    if((recieved_joint_state_r.size() > 7) && (recieved_joint_state_l.size() > 7))
    {
      joint_command_publisher_l->publish(msg1l);
      joint_command_publisher_r->publish(msg1r);
      sent = true;
    }
    else
    {
      //printf("waiting for incoming state\n");
    } 

    rclcpp::spin_some(node_r); 
    rclcpp::spin_some(node_l);

    if(sent == true)
    {
      break;
    }
    loop_rate.sleep();  
  }

  sent = false;
  usleep(10*1000000);



  // RIGHT : GO TO POINT 1----------------------------------------------------------------------------------------------
  goal_r = {42.1, -118.4, -92.1, 46.5, -143.6, 42.5, 36.1};
  ros2_control_interfaces::msg::JointControl msg2r;
  generate_command_r(&msg2r, goal_r);
  // Publish
  while (rclcpp::ok()) 
  {
    // if recieved a state from robot, update reference.
    if(recieved_joint_state_r.size() > 7)
    {
      joint_command_publisher_r->publish(msg2r);
      sent = true;
    }
    else
    {
      //printf("waiting for incoming state\n");
    } 
    rclcpp::spin_some(node_r);
    rclcpp::spin_some(node_l);
    if(sent == true)
    {
      break;
    }
    loop_rate.sleep();  
  }
  sent = false;
  usleep(4*1000000);



  // RIGHT : OPEN GRIPPER ----------------------------------------------------------------------------------------------
  grip_action_client_r->perform_grip(0);
  while (!grip_action_client_r->is_goal_done()) 
  {
    rclcpp::spin_some(grip_action_client_r);
  }



  // RIGHT : GOT TO POINT 2---------------------------------------------------------------------------------------------
  goal_r = {54.6, -118.4, -97.6, 44.9, -154.8, 60.7, 57.17};
  ros2_control_interfaces::msg::JointControl msg3r;
  generate_command_r(&msg3r, goal_r);
  // Publish
  while (rclcpp::ok()) 
  {
    // if recieved a state from robot, update reference.
    if(recieved_joint_state_r.size() > 7)
    {
      joint_command_publisher_r->publish(msg3r);
      sent = true;
    }
    else
    {
      //printf("waiting for incoming state\n");
    } 
    rclcpp::spin_some(node_r);
    rclcpp::spin_some(node_l);
    if(sent == true)
    {
      break;
    }
    loop_rate.sleep();  
  }
  sent = false;
  usleep(5*1000000);



  // RIGHT : Close GRIPPER ---------------------------------------------------------------------------------------------
  grip_action_client_r->perform_grip(100);
  while (!grip_action_client_r->is_goal_done()) 
  {
    rclcpp::spin_some(grip_action_client_r);
  }


  // RIGHT : GOT TO POINT 3---------------------------------------------------------------------------------------------
  goal_r = {66.1, -88.9, -100.6, 37.5, -35.2, 31.4, -51.1};
  ros2_control_interfaces::msg::JointControl msg4r;
  generate_command_r(&msg4r, goal_r);
  // Publish
  while (rclcpp::ok()) 
  {
    // if recieved a state from robot, update reference.
    if(recieved_joint_state_r.size() > 7)
    {
      joint_command_publisher_r->publish(msg4r);
      sent = true;
    }
    else
    {
      //printf("waiting for incoming state\n");
    } 
    rclcpp::spin_some(node_r);
    rclcpp::spin_some(node_l);

    if(sent == true)
    {
      break;
    }
    loop_rate.sleep();  
  }
  sent = false;
  usleep(5*1000000);



  // LEFT : GOT TO POINT 1----------------------------------------------------------------------------------------------
  goal_l = {-39, -104.1, 80, 8, 31, 93, 57};
  ros2_control_interfaces::msg::JointControl msg2l;
  generate_command_l(&msg2l, goal_l);
  
  // Publish
  while (rclcpp::ok()) 
  {
    // if recieved a state from robot, update reference.
    if(recieved_joint_state_l.size() > 7)
    {
      joint_command_publisher_l->publish(msg2l);
      sent = true;
    }
    else
    {
      //printf("waiting for incoming state\n");
    } 
    rclcpp::spin_some(node_r);
    rclcpp::spin_some(node_l);
    if(sent == true)
    {
      break;
    }
    loop_rate.sleep(); 
  }
  sent = false;
  usleep(10*1000000);


  // Left : Open GRIPPER ------------------------------------------------------------------------------------------------
  grip_action_client_l->perform_grip(0);
  while (!grip_action_client_l->is_goal_done()) 
  {
    rclcpp::spin_some(grip_action_client_l);
  }
  

  // LEFT : GOT TO POINT 2----------------------------------------------------------------------------------------------
  goal_l = {-56, -104, 81, 20, 35, 75, 44};
  ros2_control_interfaces::msg::JointControl msg3l;
  generate_command_l(&msg3l, goal_l);
  
  // Publish
  while (rclcpp::ok()) 
  {
    // if recieved a state from robot, update reference.
    if(recieved_joint_state_l.size() > 7)
    {
      joint_command_publisher_l->publish(msg3l);
      sent = true;
    }
    else
    {
      //printf("waiting for incoming state\n");
    } 
    rclcpp::spin_some(node_r);
    rclcpp::spin_some(node_l);

    if(sent == true)
    {
      break;
    }
    loop_rate.sleep();  
  }
  sent = false;
  usleep(5*1000000);


  // Left : Close GRIPPER ----------------------------------------------------------------------------------------------
  grip_action_client_l->perform_grip(100);
  while (!grip_action_client_l->is_goal_done()) 
  {
    rclcpp::spin_some(grip_action_client_l);
  }


  // RIGHT : OPEN GRIPPER ----------------------------------------------------------------------------------------------
  grip_action_client_r->perform_grip(0);
  while (!grip_action_client_r->is_goal_done()) 
  {
    rclcpp::spin_some(grip_action_client_r);
  }



  // LEFT : GO TO POINT 3 ----------------------------------------------------------------------------------------------
  goal_l = {-45, -104, 85, 7, 66, 49, 53};
  ros2_control_interfaces::msg::JointControl msg4l;
  generate_command_l(&msg4l, goal_l);
  
  // Publish
  while (rclcpp::ok()) 
  {
    // if recieved a state from robot, update reference.
    if(recieved_joint_state_l.size() > 7)
    {
      joint_command_publisher_l->publish(msg4l);
      sent = true;
    }
    else
    {
      //printf("waiting for incoming state\n");
    } 
    rclcpp::spin_some(node_r);
    rclcpp::spin_some(node_l);
    if(sent == true)
    {
      break;
    }
    loop_rate.sleep();  
  }
  sent = false;
  usleep(6*1000000);


  // Left : Open GRIPPER ------------------------------------------------------------------------------------------------
  grip_action_client_l->perform_grip(0);
  while (!grip_action_client_l->is_goal_done()) 
  {
    rclcpp::spin_some(grip_action_client_l);
  }


  // RIGHT : GOT TO POINT HOME------------------------------------------------------------------------------------------
  goal_r = {0.0, -130.0, -135.0, 30.0, 0.0, 30.0, 0.0};
  ros2_control_interfaces::msg::JointControl msg5r;
  generate_command_r(&msg5r, goal_r);
  // Publish
  while (rclcpp::ok()) 
  {
    // if recieved a state from robot, update reference.
    if(recieved_joint_state_r.size() > 7)
    {
      joint_command_publisher_r->publish(msg5r);
      sent = true;
    }
    else
    {
      //printf("waiting for incoming state\n");
    } 
    rclcpp::spin_some(node_r);
    rclcpp::spin_some(node_l);
    if(sent == true)
    {
      break;
    }
    loop_rate.sleep();  
  }
  sent = false;
  //usleep(10*1000000);



  // // Left : GO TO POINT 4 ----------------------------------------------------------------------------------------------
  // goal_l = {-20, -130.0, 135.0, 30.0, 10, 30.0, 20};
  // ros2_control_interfaces::msg::JointControl msg5l;  
  // generate_command_l(&msg5l, goal_l);

  // // Publish
  // while (rclcpp::ok()) 
  // {
  //   // if recieved a state from robot, update reference.
  //   if(recieved_joint_state_l.size() > 7)
  //   {
  //     joint_command_publisher_l->publish(msg5l);
  //     sent = true;
  //   }
  //   else
  //   {
  //     //printf("waiting for incoming state\n");
  //   } 
  //   rclcpp::spin_some(node_r); 
  //   rclcpp::spin_some(node_l);
  //   if(sent == true)
  //   {
  //     break;
  //   }
  //   loop_rate.sleep();  
  // }
  // sent = false;
  // usleep(3*1000000);



  // Left : Go to Point 5 ----------------------------------------------------------------------------------------------
  goal_l = {-50, -130.0, 68, 27, 35, 26, 11};
  ros2_control_interfaces::msg::JointControl msg6l;  
  generate_command_l(&msg6l, goal_l);

  // Publish
  while (rclcpp::ok()) 
  {
    // if recieved a state from robot, update reference.
    if(recieved_joint_state_l.size() > 7)
    {
      joint_command_publisher_l->publish(msg6l);
      sent = true;
    }
    else
    {
      //printf("waiting for incoming state\n");
    } 
    rclcpp::spin_some(node_r); 
    rclcpp::spin_some(node_l);
    if(sent == true)
    {
      break;
    }
    loop_rate.sleep();  
  }
  sent = false;
  usleep(2*1000000);



  // Left : Go to home pos----------------------------------------------------------------------------------------------
  goal_l = {0.0, -130.0, 135.0, 30.0, 0.0, 30.0, 0.0};
  ros2_control_interfaces::msg::JointControl msg7l;  
  generate_command_l(&msg7l, goal_l);

  // Publish
  while (rclcpp::ok()) 
  {
    // if recieved a state from robot, update reference.
    if(recieved_joint_state_l.size() > 7)
    {
      joint_command_publisher_l->publish(msg7l);
      sent = true;
    }
    else
    {
      //printf("waiting for incoming state\n");
    } 
    rclcpp::spin_some(node_r); 
    rclcpp::spin_some(node_l);
    if(sent == true)
    {
      break;
    }
    loop_rate.sleep();  
  }
  sent = false;
  usleep(10*1000000);






  // Right : Close GRIPPER ----------------------------------------------------------------------------------------------
  grip_action_client_r->perform_grip(100);
  while (!grip_action_client_r->is_goal_done()) 
  {
    rclcpp::spin_some(grip_action_client_r);
  }
  // Left : Close GRIPPER ----------------------------------------------------------------------------------------------
  grip_action_client_l->perform_grip(100);
  while (!grip_action_client_l->is_goal_done()) 
  {
    rclcpp::spin_some(grip_action_client_l);
  }






  
  //--------------------------------------------------------------------------------------------------------------------

  printf("motion completed, please ctrl+c\n");
  while(1)
  {
    usleep(1*1000000);
  }
  
  return 0;
}
