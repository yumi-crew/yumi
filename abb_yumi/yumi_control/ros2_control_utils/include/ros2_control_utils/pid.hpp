#ifndef ROS2_CONTROL_UTILS__PID_HPP
#define ROS2_CONTROL_UTILS__PID_HPP

//-- Extra --
#include <deque>
//-- End Extra --

#include <string>
#include <cmath>
#include <chrono>
#include <rclcpp/rclcpp.hpp>


namespace control_utils
{

class Pid
{
public:
  struct Gains
  {
    // Optional constructor for passing in values without antiwindup
    Gains(double p, double i, double d, double i_max, double i_min)
        : p_gain_(p),
          i_gain_(i),
          d_gain_(d),
          i_max_(i_max),
          i_min_(i_min),
          antiwindup_(false)
    {
    }
    // Optional constructor for passing in values
    Gains(double p, double i, double d, double i_max, double i_min, bool antiwindup)
        : p_gain_(p),
          i_gain_(i),
          d_gain_(d),
          i_max_(i_max),
          i_min_(i_min),
          antiwindup_(antiwindup)
    {
    }
    // Default constructor
    Gains()
        : p_gain_(0.0),
          i_gain_(0.0),
          d_gain_(0.0),
          i_max_(0.0),
          i_min_(0.0),
          antiwindup_(false)
    {
    }
    double p_gain_;   /**< Proportional gain. */
    double i_gain_;   /**< Integral gain. */
    double d_gain_;   /**< Derivative gain. */
    double i_max_;    /**< Maximum allowable integral term. */
    double i_min_;    /**< Minimum allowable integral term. */
    bool antiwindup_; /**< Antiwindup. */
  }; //end struct Gains

  /**
 * @brief Constructor, zeros out Pid values when created and
 *        initialize Pid-gains and integral term limits.
 *        Does not initialize dynamic reconfigure for PID gains
 * 
 * @param p The proportional gain.
 * @param i The integral gain.
 * @param d The derivative gain.
 * @param i_max The max integral windup.
 * @param i_min The min integral windup.
 * @param antiwindup Is anti windup enabled
 */
  Pid(double p = 0.0, double i = 0.0, double d = 0.0, double i_max = 0.0, double i_min = -0.0, bool antiwindup = false);
  Pid(const Gains &gains);

  Gains get_gains();
  void set_gains(const Gains &gains);

  double compute_command(double eaverageDeck_rror, rclcpp::Duration dt);
  double compute_command(double error, double error_dot, rclcpp::Duration dt);
  double get_current_cmd();

private:
  double p_error_last_; /**< _Save position state for derivative state calculation. */
  double p_error_;      /**< Position error. */
  double i_error_;      /**< Integral of position error. */
  double d_error_;      /**< Derivative of position error. */
  double cmd_;          /**< Command to send. */
  std::shared_ptr<Gains> gains_buffer_;

  //-- Extra --
  std::deque<double> averageDeck_;
  //-- End Extra --
};                        //end class Pid
} // namespace control_helpers

#endif