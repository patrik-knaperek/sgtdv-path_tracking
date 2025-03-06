/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

#include "speed_control.h"

int8_t SpeedControl::computeSpeedCommand(const float act_speed, const float ref_speed)
{
  ROS_DEBUG_STREAM("ref speed: " << ref_speed);
  ROS_DEBUG_STREAM("speed: " << act_speed);
  static float cmd_new = 0.f;
  static double last_raise = ros::Time::now().toSec();

  // regulation error
  const double speed_error = ref_speed - act_speed;

  // proportional
  cmd_new = params_.p * speed_error;

  // integral
  if(params_.i)
  {
    if(cmd_last_ < params_.range.max)   // Anti-windup
    {
      speed_integral_error_ += speed_error * TIME_PER_FRAME;
    }
    cmd_new += params_.i * speed_integral_error_;
  }

  // /* reference signal shaping */
  // double w = ref_speed_;
  // double w_dot = 0.;
  // double w_ddot = 0.;

  
  // /* IP controller + feed-forward correction */
  // const double e = w - act_speed;
  // speed_integral_error_ += params_.speed_i * (e + w_dot * params_.k1 + w_ddot * params_.k2);
  // cmd_new = speed_integral_error_ - params_.speed_p * act_speed;

  // saturation
  cmd_last_ = static_cast<int8_t>(std::max(params_.range.min, std::min(cmd_new, params_.range.max)));
  
  return cmd_last_;
}
