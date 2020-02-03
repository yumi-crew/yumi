#include <ros2_control_utils/pid.hpp>
#include <algorithm>

namespace control_utils
{
using namespace std::chrono_literals;

Pid::Pid(double p, double i, double d, double i_max, double i_min, bool antiwindup)
{
  gains_buffer_ = std::make_shared<Gains>(p, i, d, i_max, i_min, antiwindup);
}


Pid::Pid(const Gains &gains)
{
  gains_buffer_ = std::make_shared<Gains>(gains);
}


Pid::Gains Pid::get_gains()
{
  return *gains_buffer_;
}

void Pid::set_gains(const Pid::Gains &gains)
{
  gains_buffer_ = std::make_shared<Pid::Gains>(gains);
}

double Pid::compute_command(double error, rclcpp::Duration dt)
{

  if (dt == rclcpp::Duration(0.0) || std::isnan(error) || std::isinf(error))
  {
    return 0.0;
  }

  double error_dot = d_error_;

  // Calculate the derivative error
  if (dt > 0.0s)
  {
    error_dot = (error - p_error_last_) / dt.seconds();
    p_error_last_ = error;
  }

  //-- Extra --
  averageDeck_.push_back(error_dot);
  if(averageDeck_.size() > 6)
  {
    averageDeck_.pop_front();
  }
  double sum = 0;
  for(auto &val : averageDeck_)
  {
    sum+=val;
  }
  error_dot = sum/6;
  //-- Extra end --

  return compute_command(error, error_dot, dt);
}

double Pid::compute_command(double error, double error_dot, rclcpp::Duration dt)
{
  // Get the gain parameters from the realtime buffer
  Gains gains = *gains_buffer_;

  double p_term{0.0}, d_term{0.0}, i_term{0.0};
  p_error_ = error; // this is error = target - state
  d_error_ = error_dot;

  if (dt == rclcpp::Duration(0.0) || std::isnan(error) || std::isinf(error) || std::isnan(error_dot) || std::isinf(error_dot))
    return 0.0;

  // Calculate proportional contribution to command
  p_term = gains.p_gain_ * p_error_;

  // Calculate the integral of the position error
  if(gains.i_gain_ != 0)
  {
    i_error_ += dt.seconds() * p_error_;
  }
  
  if (gains.antiwindup_ && gains.i_gain_ != 0)
  {
    // Prevent i_error_ from climbing higher than permitted by i_max_/i_min_
    std::pair<double, double> bounds = std::minmax(gains.i_min_ / gains.i_gain_, gains.i_max_ / gains.i_gain_);
    i_error_ = std::clamp(i_error_, bounds.first, bounds.second);
  }

  // Calculate integral contribution to command
  if(gains.i_gain_ != 0)
  {
    i_term = gains.i_gain_ * i_error_;
  }

  // Calculate derivative contribution to command
  if(gains.d_gain_ != 0)
  {
    d_term = gains.d_gain_ * d_error_;
  }
  
  // Compute the command
  cmd_ = p_term + i_term + d_term;

  return cmd_;
}

double Pid::get_current_cmd()
{
  return cmd_;
}

} // namespace control_utils