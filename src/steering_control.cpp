/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

#include "steering_control.h"

/// @brief Compute steering angle command based on Pure Pursuit algorithm
/// @param msg 
/// @param steer_cmd_prev 
/// @return 
float SteeringControl::computeSteeringCommand(const sgtdv_msgs::PathTrackingMsg &msg)
{
  const auto lookahead_dist = computeLookAheadDist(msg.car_vel.speed);
  
  const Eigen::Vector2f target_point 
    = findTargetPoint(msg.trajectory.path, computeRearAxlePos(msg.car_pose), lookahead_dist);
  
  const double theta = msg.car_pose.yaw;
  const double alpha 
    = std::atan2((target_point[1] - msg.car_pose.position.y), (target_point[0] - msg.car_pose.position.x)) - theta;
  float steering_angle = static_cast<float>(std::atan2(2*std::sin(alpha)*params_.car_length,lookahead_dist));

  /* low-pass filter */
  steering_angle = params_.smooth * steering_angle + (1- params_.smooth) * cmd_last_;

  /* saturation */
  cmd_last_ = std::max(params_.range.min, std::min(params_.range.max, steering_angle));
  
  return cmd_last_;
}

float SteeringControl::computeLookAheadDist(const float speed) const
{
  float lookahead_dist = params_.k * speed;
  
  /* saturation */
  lookahead_dist = std::max(params_.lookahead_dist.min, std::min(params_.lookahead_dist.max, lookahead_dist));

  return lookahead_dist;
}

Eigen::Vector2f SteeringControl::findTargetPoint(const sgtdv_msgs::Point2DArr &trajectory,
                                              const Eigen::Vector2f &rear_axle_pos,
                                              const float lookahead_dist)
{
  const auto size = trajectory.points.size();
  static Eigen::Vector2f target_point, next_point;

  const auto closest_idx = Utils::findClosestPointIdx(rear_axle_pos, trajectory.points,
    [&](const sgtdv_msgs::Point2D p)
    {
      return Eigen::Vector2d(p.x, p.y);
    }
  );

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

Eigen::Vector2f SteeringControl::computeRearAxlePos(const sgtdv_msgs::CarPose &car_pose)
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
