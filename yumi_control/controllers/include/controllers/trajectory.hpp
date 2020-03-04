#ifndef ROS_CONTROLLERS__TRAJECTORY_HPP_
#define ROS_CONTROLLERS__TRAJECTORY_HPP_

#include <memory>
#include <vector>

#include "rclcpp/time.hpp"
#include <controllers/visibility_control.h>
#include "trajectory_msgs/msg/joint_trajectory.hpp"



namespace ros_controllers
{

using TrajectoryPointIter = std::vector<trajectory_msgs::msg::JointTrajectoryPoint>::iterator;
using TrajectoryPointConstIter = std::vector<trajectory_msgs::msg::JointTrajectoryPoint>::const_iterator;

class Trajectory
{
public:
  ROS_CONTROLLERS_PUBLIC
  Trajectory();

  ROS_CONTROLLERS_PUBLIC
  explicit Trajectory(std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory);

  ROS_CONTROLLERS_PUBLIC
  void 
  update(std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory);

  /* sample : Find the next valid point from the containing trajectory msg.
  *
  * Within each msg, points with time_from_start less or equal than current time will be skipped.
  * 
  * The first point with time_from_start greater than current time shall be a valid point,
  * Return end iterator if start time of desired trajectory msg is in the future
  * Else within each msg, valid point is the first point in the the msg with expected arrival time
  * in the future.
  * 
  * Arrival time is time_from_start of point + start time of msg
  * If an empty trajectory message is given, sample will return Trajectory::end()
  * If no valid point is found for the specified sample time, Trajectory::end() will be returned.
  */
  ROS_CONTROLLERS_PUBLIC
  TrajectoryPointConstIter
  sample(const rclcpp::Time & sample_time);

  ROS_CONTROLLERS_PUBLIC
  TrajectoryPointConstIter
  begin() const;

  ROS_CONTROLLERS_PUBLIC
  TrajectoryPointConstIter
  end() const;

  ROS_CONTROLLERS_PUBLIC
  rclcpp::Time
  time_from_start() const;

  ROS_CONTROLLERS_PUBLIC
  bool
  is_empty() const;

private:
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory_msg_;

  rclcpp::Time trajectory_start_time_;
};

}  // namespace ros_controllers

#endif  // ROS_CONTROLLERS__TRAJECTORY_HPP_