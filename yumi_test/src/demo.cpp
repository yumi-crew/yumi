#include <motion_coordinator/motion_coordinator.hpp>
#include "pose_estimation_manager.hpp"
#include <iostream>
#include <fstream>
#include <chrono>

std::shared_ptr<motion_coordinator::MotionCoordinator> yumi_motion_coordinator;
std::shared_ptr<PoseEstimationManager> pose_estimation_manager;

using namespace std::chrono_literals;

// Ctr+C handler
void signal_callback_handler(int signum)
{
  std::cout << "Caught signal " << signum << std::endl;
  // Terminate EGM session
  yumi_motion_coordinator->terminate_egm_session();
  // shutdown lifecycle nodes
  pose_estimation_manager->change_state(
      "zivid_camera", lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, 30s);
  pose_estimation_manager->change_state(
      "pose_estimation", lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, 30s);
  pose_estimation_manager->change_state(
      "zivid_camera", lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP, 30s);
  pose_estimation_manager->change_state(
      "pose_estimation", lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP, 30s);
  // Terminate ros node
  rclcpp::shutdown();
  // Terminate program
  exit(signum);
}

void spin(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exe)
{
  exe->spin();
}

int main(int argc, char **argv)
{
  // Ctrl+C handler
  signal(SIGINT, signal_callback_handler);

  rclcpp::init(argc, argv);

  // Initialize yumi motion coordinator
  yumi_motion_coordinator = std::make_shared<motion_coordinator::MotionCoordinator>("motion_coordinator");
  if (!yumi_motion_coordinator->init())
  {
    std::cout << "[ERROR]: MotionCoordinator failed to initialize." << std::endl;
    return -1;
  }

  // Spin the moveit2 node countiously in another thread via an executor
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(yumi_motion_coordinator->get_node());
  auto future_handle = std::async(std::launch::async, spin, executor);

  // Activate yumi, EGM, Moveit2
  if (!yumi_motion_coordinator->activate())
  {
    std::cout << "[ERROR]: MotionCoordinator failed to activate." << std::endl;
    return -1;
  }

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
  pose_estimation_manager->add_camera_parameter("zivid_camera.capture.frame_2.iris", rclcpp::ParameterValue(38));
  pose_estimation_manager->add_camera_parameter("zivid_camera.capture.frame_2.gain", rclcpp::ParameterValue(3.21));
  pose_estimation_manager->add_camera_parameter("zivid_camera.capture.frame_3.iris", rclcpp::ParameterValue(42));
  pose_estimation_manager->add_camera_parameter("zivid_camera.capture.frame_3.gain", rclcpp::ParameterValue(4.16));
  pose_estimation_manager->add_camera_parameter("zivid_camera.capture.general.filters.reflection.enabled", rclcpp::ParameterValue(true));
  pose_estimation_manager->call_set_param_srv(30s);

  auto state3 = pose_estimation_manager->get_state("zivid_camera", 30s);
  auto state4 = pose_estimation_manager->get_state("pose_estimation", 30s);

  auto transition_success4 = pose_estimation_manager->change_state(
      "pose_estimation", lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, 30s);
  auto state5 = pose_estimation_manager->get_state("zivid_camera", 30s);
  auto state6 = pose_estimation_manager->get_state("pose_estimation", 30s);

  char *buf = getlogin();
  std::string u_name = buf;

  pose_estimation_manager->call_init_halcon_surface_match_srv("/home/" + u_name + "/abb_ws/src/object_files/ply/", 500s);
  auto transition_success3 = pose_estimation_manager->change_state(
      "zivid_camera", lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, 30s);


  int ret_val;
  int counter = 0;
  int tries = 0;
  int num_picks = 10;
  bool first = true;
  bool cap_success{false};
  bool est_success{false};
  std::string arm = "right_arm";
  std::vector<std::string> objects = {"small_marker", "nail_polish", "battery"};
  double percentage = 0;
  int lin_retries = 3;

  // Move arms to setup pos.
  yumi_motion_coordinator->move_to_home(arm, 3);
  yumi_motion_coordinator->move_to_home("left_arm", 3);
  

  cap_success = pose_estimation_manager->call_capture_srv(30s);
  sleep(5);
  // Find and add place bin to scene
  std::string place_bin = "bin5";
  est_success = pose_estimation_manager->call_estimate_pose_srv(place_bin, 0, 100s, "", 0.0, true);
  auto place_bin_pose = pose_estimation_manager->pose_transformer->obj_in_base_frame();
  yumi_motion_coordinator->add_object(place_bin, place_bin_pose, false, {1.0, 0.64, 0.0, 1}); //Pink

  // Find and add pick bin bin to scene
  std::string pick_bin = "bin6";
  est_success = pose_estimation_manager->call_estimate_pose_srv(pick_bin, 0, 100s, "inliers", 0.1, true);
  auto pick_bin_pose = pose_estimation_manager->pose_transformer->obj_in_base_frame();
  yumi_motion_coordinator->add_object(pick_bin, pick_bin_pose, false, {0, 0, 1, 1});    //Blue

  std::map<int, std::string> errors;
  errors[0] = "SUCCESS";
  errors[-1] = "INVALID_POSE";
  errors[-2] = "LINEAR_PLAN_FAIL";
  errors[-3] = "GRIP_FAIL";
  errors[-4] = "PLANNING_SCENE_FAIL";
  std::map<std::string, int> exp_log;
  for (auto [key, value] : errors)
  {
    exp_log[value] = 0;
  }
  exp_log["POSE_ESTIMATION_FAIL"] = 0;

  std::vector<double> cycle_times;
  while (counter < num_picks)
  {
    for (auto object : objects)
    {
      auto cycle_start = std::chrono::high_resolution_clock::now();

      tries++;
      // Move away from the camera view
      //yumi_motion_coordinator->move_to_home(arm, 5);
      yumi_motion_coordinator->move_to_state(arm, {0.43, -1.32, -0.61, 0.64, -1.44, 1.27, -0.07});

      // Take image
      std::cout << "before call_capture_srv" << std::endl;
      cap_success = pose_estimation_manager->call_capture_srv(30s);

      // Find the boject's pose in the the camera frame
      std::cout << "before call_estimate_pose_srv" << std::endl;
      if (!pose_estimation_manager->call_estimate_pose_srv(object, 1, 50s, "outliers", 0.12))
      {
        exp_log["POSE_ESTIMATION_FAIL"]++;
        std::cout << "[ERROR] object cannot be found." << std::endl;
        if (yumi_motion_coordinator->object_present(object))
          yumi_motion_coordinator->remove_object(object);
        continue;
      }

      // Find the object's pose in the base frame of the robot
      auto grasp_pose = pose_estimation_manager->pose_transformer->obj_in_base_frame();

      // Add red mesh to detected object
      if (!yumi_motion_coordinator->object_present(object))
        yumi_motion_coordinator->add_object(object, grasp_pose, false, {1, 0, 0, 1});
      else
        yumi_motion_coordinator->move_object(object, grasp_pose);

      // Pick object
      ret_val = yumi_motion_coordinator->pick_object(arm, object, {"table", pick_bin}, lin_retries, 0.10, true, false, percentage);
      if (ret_val < 0)
      {
        exp_log[errors[ret_val]]++;
        std::cout << "pick failed" << std::endl;
        continue;
      }

      // Place at a object
      ret_val = yumi_motion_coordinator->place_at_object(arm, place_bin, lin_retries, 0.15, true, false, percentage);
      if (ret_val < 0)
      {
        exp_log[errors[ret_val]]++;
        std::cout << "place failed" << std::endl;
        continue;
      }
      counter++; // If place return true the object must be dropped at the destination and can thus be considered successfully picked and placed.
      exp_log[errors[0]]++;
      auto cycle_end = std::chrono::high_resolution_clock::now();
      auto cycle_time = std::chrono::duration_cast<std::chrono::milliseconds>(cycle_end - cycle_start);
      cycle_times.push_back((double)cycle_time.count()/1000.0);
      if (counter >= num_picks)
        break;
    }
  }

  // go to home
  yumi_motion_coordinator->move_to_home(arm, 3);

  std::fstream log_file;
  log_file.open("log_file.txt", std::fstream::out);
  log_file << "Number of pics completed: " << num_picks << std::endl;
  log_file << "Number of tries: " << tries << std::endl;
  for (auto [error, value] : exp_log)
  {
    log_file << error << ": " << value << std::endl;
  }
  log_file << "Cycle times: ";
  double sum{0.0};
  for(auto ct : cycle_times)
  {
    log_file << ct << ", ";
    sum += ct;
  }
  log_file << std::endl << "Average cycle time: " << sum / tries << std::endl;

  log_file.close();
  std::cout << "Motion completed, please ctrl+c" << std::endl;
  while (1)
  {
    sleep(1);
  }
  return 0;
}
