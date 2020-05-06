#include <motion_coordinator/motion_coordinator.hpp>
#include "pose_estimation_manager.hpp"

std::shared_ptr<motion_coordinator::MotionCoordinator> yumi_motion_coordinator;
std::shared_ptr<PoseEstimationManager> pose_estimation_manager; 

using namespace std::chrono_literals;

// Ctr+C handler
void signal_callback_handler(int signum)
{
  std::cout << "Caught signal " << signum << std::endl;
   // shutdown lifecycle nodes
  pose_estimation_manager->change_state(
      "zivid_camera", lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, 30s);
  pose_estimation_manager->change_state(
      "pose_estimation", lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, 30s);
  pose_estimation_manager->change_state(
      "zivid_camera", lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP, 30s);
  pose_estimation_manager->change_state(
      "pose_estimation", lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP, 30s);
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
  rclcpp::init(argc, argv);

  yumi_motion_coordinator = std::make_shared<motion_coordinator::MotionCoordinator>("onsket_nodenavn");
  if(!yumi_motion_coordinator->init()) return -1;

  // Spin the moveit2 node countiously in another thread via an executor
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(yumi_motion_coordinator->get_node());
  auto future_handle = std::async(std::launch::async, spin, executor);



  // zivid + pose_estimation
  pose_estimation_manager = std::make_shared<PoseEstimationManager>("pose_estimation_manager");

  rclcpp::executors::MultiThreadedExecutor exe;
  exe.add_node(pose_estimation_manager);


  auto state1 = pose_estimation_manager->get_state("zivid_camera", 10s);
  auto state2 = pose_estimation_manager->get_state("pose_estimation", 10s);

  auto transition_success1 = pose_estimation_manager->change_state(
      "zivid_camera", lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, 30s);
  auto transition_success2 = pose_estimation_manager->change_state(
      "pose_estimation", lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, 30s);

  pose_estimation_manager->add_camera_parameter("zivid_camera.capture.frame_0.iris", rclcpp::ParameterValue(17));
  pose_estimation_manager->add_camera_parameter("zivid_camera.capture.frame_1.iris", rclcpp::ParameterValue(25));
  pose_estimation_manager->add_camera_parameter("zivid_camera.capture.frame_2.iris", rclcpp::ParameterValue(30));
  pose_estimation_manager->call_set_param_srv(30s);

  auto state3 = pose_estimation_manager->get_state("zivid_camera", 30s);
  auto state4 = pose_estimation_manager->get_state("pose_estimation", 30s);

  auto transition_success4 = pose_estimation_manager->change_state(
      "pose_estimation", lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, 30s);
  auto state5 = pose_estimation_manager->get_state("zivid_camera", 30s);
  auto state6 = pose_estimation_manager->get_state("pose_estimation", 30s);

  pose_estimation_manager->call_init_halcon_surface_match_srv("/home/students/abb_ws/src/object_files/ply/", 2, 500s);
  auto transition_success3 = pose_estimation_manager->change_state(
      "zivid_camera", lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, 30s);
  


  // Activate yumi, EGM, Moveit2
  if(!yumi_motion_coordinator->activate())
  {
    std::cout << "ERROR: MotionCoordinator failed to activate." << std::endl;
    return -1;  
  }

  bool first = true;
  bool cap_success{false};
  bool est_success{false};
  std::vector<double> bin_pose = {0.4, -0.1, -0.1, -0.1, 0, 180 };
  
  yumi_motion_coordinator->add_object("bin", bin_pose, true);
  
  
  while(!yumi_motion_coordinator->should_stop())
  {
    // Go to home
    yumi_motion_coordinator->move_to_home("left_arm", 3, false, true, false);

    // Tar bilde
    std::cout << "before call_capture_srv" << std::endl;
    cap_success = pose_estimation_manager->call_capture_srv(30s);

    // Finn pose i camera frame
    std::cout << "before call_estimate_pose_srv" << std::endl;
    est_success = pose_estimation_manager->call_estimate_pose_srv("screwdriver", 50s);

    // Finn pose i base frame
    auto grasp_pose = pose_estimation_manager->pose_transformer->obj_in_base_frame();

    // Legg mesh på object
    if(first) yumi_motion_coordinator->add_object("screwdriver", grasp_pose, false);
    else yumi_motion_coordinator->move_object("screwdriver", grasp_pose);
     
    // Plukk
    if(!yumi_motion_coordinator->pick_object("left_arm", "screwdriver", 3, 0.15, true, false, 0.8)){ std::cout << "pick failed" << std::endl; }
 
    // Place
    if(!yumi_motion_coordinator->place_at_object("left_arm", "bin", 3, 0.15, true, false, 0.8)){ std::cout << "place failed" << std::endl; } 

    first = false;
  }

  
  std::cout << "Motion completed, please ctrl+c" << std::endl;
  while(1)
  {
    sleep(1);
  }
  return 0;
}
