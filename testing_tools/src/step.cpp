
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>
#include <unistd.h>
#include <fstream>
#include <math.h> 

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include <ros2_control_interfaces/msg/joint_control.hpp>

#include <angles/angles.h>

#include <rws_clients/robot_manager_client.hpp>

#include <yumi_robot_manager_interfaces/srv/is_ready.hpp>
#include <yumi_robot_manager_interfaces/srv/stop_egm.hpp>
#include <yumi_robot_manager_interfaces/srv/start_egm.hpp>




/*    TESTING TOOLS - STEP
 * 
 *    Sends a joint goal as a step
 * 
 *    Effectively asking the joint to follow a step
 */


// Global div
int joint_nr;
std::array<std::string, 7> joint_names;
std::string node_name_rmc = "robot_manager_client";
auto robot_manager_client = std::make_shared<rws_clients::RobotManagerClient>(node_name_rmc);
bool first_callback = true;
bool should_log = false;

// For plotting
typedef struct Point
{
  double timestamp;
  double joint_pos;
}
Point;
std::vector<Point> cur_pos;
std::vector<Point> des_pos;

// CTRL+C handler
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

// For plotting
std::vector<double> recieved_joint_state_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
std::vector<double> initial_pos;
double last_command;
double last_recieved;
double cur_pos_timestamp;
double des_pos_timestamp;


// For sine wave math
double a = 20;   // amplitude
double f = 0.2;  // frequency (sine wave)
double p = 0;    // phase
double one_wave_time = 1.0/f;

bool test_over = false;
// Global timekeeping
rclcpp::Clock::SharedPtr klokke = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
auto start = klokke->now().seconds();


// Subscription callback
void callback(sensor_msgs::msg::JointState::UniquePtr msg)
{
  for(int i = 0; i < 7; i++)
  {
    recieved_joint_state_[i] = msg->position[i];
  }
  if(first_callback)
  {
    initial_pos = recieved_joint_state_;
    first_callback = false;
  }

  // For plotting
  if(should_log)
  {
    last_recieved = recieved_joint_state_[joint_nr];
    cur_pos_timestamp = (double)( msg->header.stamp.sec + (double)msg->header.stamp.nanosec/1000000000.0 );
  }
}

// Command generator
void generate_command(ros2_control_interfaces::msg::JointControl *cmd_msg)
{
  auto now = klokke->now().seconds();
  auto t = now - start;
  
  for(int i = 0; i < 7; ++i)
  {
    cmd_msg->joints.push_back(joint_names[i]);
    cmd_msg->goals.push_back(initial_pos[i]);
  }
  cmd_msg->header.stamp = rclcpp::Clock().now();
  cmd_msg->goals[joint_nr] += angles::from_degrees(10);

  // For plotting
  if(!should_log)
  {
    should_log = true;
  }
  last_command = cmd_msg->goals[joint_nr];
  des_pos_timestamp = (double)(cmd_msg->header.stamp.sec + (double)cmd_msg->header.stamp.nanosec/1000000000.0);
}


void spin(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exe)
{
  exe->spin();
}


std::vector<Point> remove_zero_rows(std::vector<Point> point_list)
{
  std::vector<Point> filtered;
  for(auto x : point_list)
  {
    if(  (x.joint_pos != 0) )
    {
      Point to_add;
      to_add.timestamp = x.timestamp;
      to_add.joint_pos = x.joint_pos;
      filtered.push_back(to_add);
    }
  }
  return filtered;
}



int main(int argc, char * argv[])
{
  // Ctrl+C handler
  signal(SIGINT, signal_callback_handler);

  rclcpp::init(argc, argv);

  // making node
  auto node_s = rclcpp::Node::make_shared("sine_wave_s");
  auto node_p = rclcpp::Node::make_shared("sine_wave_p");
  std::string nodegroup_namespace = node_p->get_namespace();
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
  // Start EGM mode
  if(!robot_manager_client->start_egm())
  {
    printf("Failed to start egm \n");
  }

  
  // creating publisher
  auto joint_command_publisher = node_p->create_publisher<ros2_control_interfaces::msg::JointControl>(nodegroup_namespace+"/joint_commands", 10);
  // creating subscription
  auto joint_state_subscription = node_s->create_subscription<sensor_msgs::msg::JointState>(nodegroup_namespace+"/joint_states", 10, callback);
  
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node_s);
  auto future_handle = std::async(std::launch::async, spin, executor);
  usleep(2*1000000);
  


  // (PLOT) Construct points
  Point point_cur;
  Point point_des;


  bool not_sendt = true;

  // publishing rate at 250 hz.
  rclcpp::WallRate loop_rate(250);
   // Start time is now
  start = klokke->now().seconds();
  while (rclcpp::ok()) 
  {
    // (PLOT) Save recieved state with timestamp (timestamp from msg generation)
    point_cur.joint_pos = last_recieved;
    point_cur.timestamp = cur_pos_timestamp;

    // Perform step after 1.0 seconds
    auto timepassed = klokke->now().seconds() - start;
    if( (timepassed >= 1.0) && not_sendt )
    { 
      //Publish command
      ros2_control_interfaces::msg::JointControl msg;
      generate_command(&msg);
      joint_command_publisher->publish(msg);

      rclcpp::spin_some(node_p);
      not_sendt = false;
    }

    // (PLOT) add to vector
    cur_pos.push_back(point_cur);  

    // Reference considered reached if within 0.1 degrees
    if( (timepassed >= 2.6) && (std::abs(std::abs(last_recieved)-std::abs(last_command)) < 0.0017) )
    {
      printf("timepassed: %f\n", timepassed);
      executor->cancel();
      break;
    }

    loop_rate.sleep();
  }
    
  for(auto x : initial_pos)
  {
    printf("initial_pos[i] = %f\n", x);
  }

  // scale the timestamps to nearest one-hundreth of a ms
  double scale = 0.0001;  // i.e. round to nearest one-hundreth of a ms
  // Remove any zero rows from cur_pos
  cur_pos = remove_zero_rows(cur_pos);
  double cur_start_time = floor( start  / scale + 0.5)*scale; 
  double des_start_time = floor( des_pos_timestamp  / scale + 0.5)*scale; 


  for(int i = 0; i < cur_pos.size(); ++i)
  {
    printf("cur_pos[i].timestamp: %f\n", cur_pos[i].timestamp);
    printf("cur_pos[i].joint_pos: %f\n", cur_pos[i].joint_pos);
  }
  

  // (PLOT) save to files
  std::ofstream ofs_des("step_des.txt"); //desired
  ofs_des << 0.0 << "$" << initial_pos[joint_nr] << std::endl;
  ofs_des << floor(  des_pos_timestamp / scale + 0.5)*scale - cur_start_time<< "$" << last_command << std::endl;
  

  std::ofstream ofs_cur("step_cur.txt"); //current
  for(int i = 0; i < cur_pos.size(); ++i)
  {
    ofs_cur << floor( cur_pos[i].timestamp  / scale + 0.5)*scale - cur_start_time << "$" << cur_pos[i].joint_pos << std::endl;
  }
    

  // keep running until ctrl+c
  printf("motion completed, please ctrl+c\n");
  while(1)
  {
    usleep(1*1000000);
  }


  rclcpp::shutdown();
  return 0;
}
