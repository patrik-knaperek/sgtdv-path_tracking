/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

#include "speed_control.h"

int8_t SpeedControl::computeSpeedCommand(const float act_speed, const float ref_speed)
{
  ROS_DEBUG_STREAM("ref speed: " << ref_speed);
  ROS_DEBUG_STREAM("speed: " << act_speed);

  double cmd_new;
  static int8_t cmd_last = 0.f;

  /* reference signal shaping */ 
  {  
    auto j1 = params_.K1 * (static_cast<double>(ref_speed) - w_ - w_dot_);
    j1 = Utils::saturation(j1, -params_.j_limit, params_.j_limit);

    w_ddot_ = j1 - aw_;
    jerk_integral_ += w_ddot_ * TIME_PER_FRAME;
    
    const auto a1 = params_.K2 * jerk_integral_;
    w_dot_ = Utils::saturation(a1, -params_.a_limit, params_.a_limit);
    
    aw_ = std::max((a1 - w_dot_) * FPS, 0.);

    w_ = acc_integral_ += w_dot_ * TIME_PER_FRAME;
    ROS_DEBUG_STREAM("w = " << w_);
  }

  if(params_.type == 1)
  {
    /* IP controller + feed-forward correction */
    static double K_p = (2 * params_.b * params_.w0 * params_.T - 1) / params_.K;
    static double K_i = std::pow(params_.w0,2) * params_.T / params_.K;
    static double k1 = (K_p * params_.K+1)/(K_i * params_.K);
    static double k2 = params_.T / (K_i * params_.K);

    const double error = w_ - static_cast<double>(act_speed);
    const double feed_forward = w_dot_ * k1 + w_ddot_ * k2;
    speed_integral_ += K_i * error + feed_forward;
    cmd_new = speed_integral_ - K_p * act_speed;
  }
  else
  { /* basic IP regulator */
    // regulation error
    const double speed_error = w_ - act_speed;

    // proportional
    cmd_new = params_.p * speed_error;

    // integral
    if(params_.i)
    {
      if(cmd_last < params_.cmd_range.max)   // Anti-windup
      {
        speed_integral_ += speed_error * TIME_PER_FRAME;
      }
      cmd_new += params_.i * speed_integral_;
    }
  }

  ROS_DEBUG_STREAM("cmd new = " << cmd_new);

  cmd_last = static_cast<int8_t>(Utils::saturation(cmd_new, params_.cmd_range.min, params_.cmd_range.max));
  
  return cmd_last;
}

void SpeedControl::reinit()
{
  w_ = w_dot_ = w_ddot_ = 0.;
  aw_ = 0.;
  jerk_integral_ = acc_integral_ = speed_integral_ = 0.;
}
