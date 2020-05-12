#include "external_force/external_force.hpp"
#include "kdl_wrapper/kdl_wrapper.h"
#include <urdf/model.h>
#include <rclcpp/rclcpp.hpp>
#include <future>

#include <unistd.h>

// Ctr+C handler
void signal_callback_handler(int signum)
{
  std::cout << "Caught signal " << signum << std::endl;
  // Terminate ros node
  rclcpp::shutdown();
  // Terminate program
  exit(signum);
}

int main(int argc, char *argv[])
{
  // Ctrl+C handler
  signal(SIGINT, signal_callback_handler);

  rclcpp::init(argc, argv);
  // Finding path to urdf
  char buf[20];
  if (getlogin_r(buf, 20) != 0)
  {
    printf("Unable to find username\n");
    return -1;
  }
  std::string username = buf;
  std::string path = "/home/" + username + "/abb_ws/src/yumi/yumi_description/urdf/yumi.urdf";

  // Loading URDf and constructing the KdlWrapper
  urdf::Model robot_model;
  if (!robot_model.initFile(path))
  {
    printf("unable to load urdf\n");
    return -1;
  }
  
  auto ext_F = std::make_shared<yumi_dynamics::ExternalForce>(robot_model);
  rclcpp::WallRate loop_rate(250);
  while (rclcpp::ok())
  {
    ext_F->estimate_TCP_wrench();
    loop_rate.sleep();
  }


  return 0;
}
