/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/

#include "path_tracking.h"

void PathTracking::update(const sgtdv_msgs::PathTrackingMsg &msg, sgtdv_msgs::Control *cmd)
{      
  cmd->steering_angle = computeSteeringCommand(msg, cmd->steering_angle);

  cmd->speed = computeSpeedCommand(msg.car_vel.speed, cmd->speed);
}


int8_t PathTracking::computeSpeedCommand(const float act_speed, const int8_t speed_cmd_prev)
{
  ROS_DEBUG_STREAM("ref speed: " << ref_speed_);
  ROS_DEBUG_STREAM("speed: " << act_speed);
  static float speed_cmd_act = 0.f;
  static double last_raise = ros::Time::now().toSec();

  // regulation error
  const double speed_error = ref_speed_ - act_speed;

  // proportional
  speed_cmd_act = params_.speed_p * speed_error;

  // integral
  if(params_.speed_i)
  {
    if(speed_cmd_prev < params_.speed.max)   // Anti-windup
    {
      speed_integral_error_ += speed_error * TIME_PER_FRAME;
    }
    speed_cmd_act += params_.speed_i * speed_integral_error_;
  }

  // ramp
  // const double speedCmdDelta = speed_cmd_act - speed_cmd_prev;
  // if(std::abs(speedCmdDelta) > 1.0)
  // {
  //   if((ros::Time::now().toSec() - last_raise) > 1. / params_.speed_raise_rate)
  //   {
  //     speed_cmd_act = speed_cmd_prev + (speedCmdDelta) / std::abs(speedCmdDelta);
  //     last_raise = ros::Time::now().toSec();
  //   }
  //   else
  //   {
  //     speed_cmd_act = speed_cmd_prev;
  //   }
  // }
  
  // saturation
  speed_cmd_act = std::max(params_.speed.min, std::min(speed_cmd_act, params_.speed.max));
  
  return static_cast<int8_t>(speed_cmd_act);
}

float PathTracking::computeSteeringCommand(const sgtdv_msgs::PathTrackingMsg &msg, const float steer_cmd_prev)
{
  const auto lookahead_dist = computeLookAheadDist(msg.car_vel.speed);
  
  const Eigen::Vector2f target_point 
    = findTargetPoint(msg.trajectory, computeRearAxlePos(msg.car_pose), lookahead_dist);
  
  const double theta = msg.car_pose.yaw;
  const double alpha 
    = std::atan2((target_point[1] - msg.car_pose.position.y), (target_point[0] - msg.car_pose.position.x)) - theta;
  float steering_angle = static_cast<float>(std::atan2(2*std::sin(alpha)*params_.car_length,lookahead_dist));

  /* low-pass filter */
  steering_angle = params_.steering_smooth * steering_angle + (1- params_.steering_smooth) * steer_cmd_prev;

  /* saturation */
  steering_angle = std::max(params_.steering.min, std::min(params_.steering.max, steering_angle));
  
  return steering_angle;
}

float PathTracking::computeLookAheadDist(const float speed) const
{
  float lookahead_dist = params_.steering_k * speed;
  
  /* saturation */
  lookahead_dist = std::max(params_.lookahead_dist.min, std::min(params_.lookahead_dist.max, lookahead_dist));

  return lookahead_dist;
}

Eigen::Vector2f PathTracking::findTargetPoint(const sgtdv_msgs::Point2DArr &trajectory,
                                              const Eigen::Vector2f &rear_axle_pos,
                                              const float lookahead_dist)
{
  const auto size = trajectory.points.size();
  static Eigen::Vector2f target_point, next_point;

  const auto closest_idx = findClosestPointIdx(trajectory, rear_axle_pos);

  int offset(0), next_idx;
  target_point << trajectory.points[closest_idx].x,
                  trajectory.points[closest_idx].y;
  
  while(true)
  {
    if(!params_.track_loop)
    {
      next_idx = (closest_idx + offset++);
      if(next_idx > size - 1) break;
    }
    else
    {
      next_idx = (closest_idx + offset++) % size;
    }
    next_point << trajectory.points[next_idx].x,
                  trajectory.points[next_idx].y;

    if((rear_axle_pos - next_point).norm() < lookahead_dist)
    {
      target_point = next_point;
      continue;
    }
    else
    {
      break;
    }
  }

  /* compute goal position exactly in look-ahead distance from the vehicle's position,
  *  interpolated on line given by the trajectory points 
  */
  if(target_point != next_point)
  {
    const auto slope_angle = std::atan2(
      next_point(1) - target_point(1),
      next_point(0) - target_point(0)
    );
    
    const auto bearing_vector = target_point - rear_axle_pos;
    const auto d = bearing_vector.norm();
    const auto theta = std::atan2(bearing_vector(1), bearing_vector(0));
    const auto gamma = M_PI - slope_angle + theta;

    const auto x = 
      d * cos(gamma) + std::sqrt(std::pow(d,2) * std::pow(cos(gamma),2) - std::pow(d,2) + std::pow(lookahead_dist,2));

    target_point(0) += cos(slope_angle) * x,
    target_point(1) += sin(slope_angle) * x;
  }
#ifdef SGT_VISUALIZATION
  pure_pursuit_points_ = std::make_pair(
    target_point, Eigen::Vector2f(trajectory.points[closest_idx].x, trajectory.points[closest_idx].y)
  );
#endif /* SGT_VISUALIZATION */

  return target_point;
}

/// @brief Finds an index of the closest trajectory point to the reference position
/// @param trajectory - array of the trajecotry points
/// @param pos - reference position
/// @return index of the trajectory point in the array
size_t PathTracking::findClosestPointIdx(const sgtdv_msgs::Point2DArr &trajectory, 
                                          const Eigen::Vector2f& pos) const
{
  const auto trajectory_it  = std::min_element(trajectory.points.begin(), trajectory.points.end(),
                                              [&](const sgtdv_msgs::Point2D &a,
                                              const sgtdv_msgs::Point2D &b) {
                                              const double da = std::hypot(pos[0] - a.x,
                                                                          pos[1] - a.y);
                                              const double db = std::hypot(pos[0] - b.x,
                                                                          pos[1] - b.y);

                                              return da < db;
                                              });
  return std::distance(trajectory.points.begin(), trajectory_it);
}

Eigen::Vector2f PathTracking::computeRearAxlePos(const sgtdv_msgs::CarPose &car_pose)
{
  const Eigen::Vector2f pos(car_pose.position.x, car_pose.position.y);
  const Eigen::Vector2f rear_axle_pos = pos - Eigen::Vector2f(cosf(car_pose.yaw) * params_.rear_wheels_offset, 
                                          sinf(car_pose.yaw)  * params_.rear_wheels_offset);

#ifdef SGT_VISUALIZATION
  axle_pos_.first = rear_axle_pos;
  axle_pos_.second = pos + Eigen::Vector2f(cosf(car_pose.yaw) * params_.front_wheels_offset, // front axle position
                      sinf(car_pose.yaw) * params_.front_wheels_offset);
#endif /* SGT_VISUALIZATION */

return rear_axle_pos;
}
