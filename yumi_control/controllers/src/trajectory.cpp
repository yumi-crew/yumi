
#include <controllers/trajectory.hpp>
#include <memory>
#include "hardware_interface/macros.hpp"
#include "hardware_interface/utils/time_utils.hpp"
#include "rclcpp/clock.hpp"

namespace ros_controllers
{

using hardware_interface::utils::time_is_zero;
using hardware_interface::utils::time_less_than_equal;
using hardware_interface::utils::time_add;
using hardware_interface::utils::time_less_than;

Trajectory::Trajectory()
: trajectory_start_time_(0)
{}

Trajectory::Trajectory(std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory)
: trajectory_msg_(joint_trajectory),
  trajectory_start_time_(time_is_zero(joint_trajectory->header.stamp) ?  //if
    rclcpp::Clock().now() : //then
    static_cast<rclcpp::Time>(joint_trajectory->header.stamp)) //else
{}

void
Trajectory::update(std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory)
{
  trajectory_msg_ = joint_trajectory;
  trajectory_start_time_ = (time_is_zero(joint_trajectory->header.stamp) ? //if
    rclcpp::Clock().now() :  //then
    static_cast<rclcpp::Time>(joint_trajectory->header.stamp));  //else
}

TrajectoryPointConstIter
Trajectory::sample(const rclcpp::Time & sample_time)
{
  // skip if current time hasn't reached traj time of the first msg yet
  if (time_less_than(sample_time, trajectory_start_time_)) 
  {
    return end();
  }

  // time_from_start + trajectory time is the expected arrival time of trajectory
  for (auto point = begin(); point != end(); ++point) 
  {
    auto start_time = time_add(trajectory_start_time_, point->time_from_start);
    if (time_less_than(sample_time, start_time)) 
    {
      return point;
    }
  }

  return end();
}

TrajectoryPointConstIter
Trajectory::begin() const
{
  // THROW_ON_NULLPTR(trajectory_msg_)

  return trajectory_msg_->points.begin();
}

TrajectoryPointConstIter
Trajectory::end() const
{
  // THROW_ON_NULLPTR(trajectory_msg_)

  return trajectory_msg_->points.end();
}

rclcpp::Time
Trajectory::time_from_start() const
{
  return trajectory_start_time_;
}

bool
Trajectory::is_empty() const
{
  return !trajectory_msg_;
}

}  // namespace ros_controllers