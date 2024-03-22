/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Tereza Ábelová, Juraj Krasňanský, Patrik Knaperek
/*****************************************************/


#include "../include/tracking_algorithms.h"

TrackingAlgorithm::TrackingAlgorithm(const ros::NodeHandle &handle
#ifdef SGT_VISUALIZATION
  , const ros::Publisher& target_pub
  , const ros::Publisher& steering_pose_pub
) :
  target_pub_(target_pub),
  steering_pose_pub_(steering_pose_pub)
#else
)
#endif /* SGT_VISUALIZATION */
{
}

int8_t TrackingAlgorithm::computeSpeedCommand(const float act_speed, const int8_t speed_cmd_prev)
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
    if(speed_cmd_prev < params_.speed_max)   // Anti-windup
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
  speed_cmd_act = std::max(params_.speed_min, std::min(speed_cmd_act, params_.speed_max));
  
  return static_cast<int8_t>(speed_cmd_act);
}

/// @brief Finds an index of the closest trajectory point to the reference position
/// @param trajectory - array of the trajecotry points
/// @param pos - reference position
/// @return index of the trajectory point in the array
size_t TrackingAlgorithm::findClosestPointIdx(const sgtdv_msgs::Point2DArr::ConstPtr &trajectory, 
                                              const Eigen::Ref<const Eigen::Vector2f>& pos) const
{
  const auto trajectory_it  = std::min_element(trajectory->points.begin(), trajectory->points.end(),
                                              [&](const sgtdv_msgs::Point2D &a,
                                              const sgtdv_msgs::Point2D &b) {
                                              const double da = std::hypot(pos[0] - a.x,
                                                                          pos[1] - a.y);
                                              const double db = std::hypot(pos[0] - b.x,
                                                                          pos[1] - b.y);

                                              return da < db;
                                              });
  return std::distance(trajectory->points.begin(), trajectory_it);
}

#ifdef SGT_VISUALIZATION
void TrackingAlgorithm::visualizePoint(const Eigen::Ref<const Eigen::Vector2f>& point, const int point_id, 
                              const std::string& ns, const Eigen::Ref<const Eigen::Vector3f> color) const
{
  visualization_msgs::Marker marker;
  
  marker.color.r              = color(0);
  marker.color.g              = color(1);
  marker.color.b              = color(2);
  marker.color.a              = 1.0;
  marker.pose.position.x      = point[0];
  marker.pose.position.y      = point[1];
  marker.pose.orientation.w   = 1.0;
  marker.type                 = visualization_msgs::Marker::SPHERE;
  marker.action               = visualization_msgs::Marker::ADD;
  marker.id                   = point_id;
  marker.ns                   = ns;
  marker.scale.x              = 0.3;
  marker.scale.y              = 0.3;
  marker.scale.z              = 0.3;
  marker.header.stamp         = ros::Time::now();
  marker.header.frame_id      = "map";
  target_pub_.publish(marker);
}

void TrackingAlgorithm::visualizeSteering() const
{
  geometry_msgs::PoseStamped steering_pose;
  steering_pose.header.stamp = ros::Time::now();
  steering_pose.header.frame_id = std::string("base_link");
  steering_pose.pose.position.x  = params_.front_wheels_offset;

  steering_pose.pose.orientation.z = sin((control_.steering_angle) / 2);
  steering_pose.pose.orientation.w = cos((control_.steering_angle) / 2);

  steering_pose_pub_.publish(steering_pose);
}
#endif /* SGT_VISUALIZATION */

PurePursuit::PurePursuit(const ros::NodeHandle& handle
  #ifdef SGT_VISUALIZATION
    , const ros::Publisher& target_pub
    , const ros::Publisher& steering_pose_pub
  #endif /* SGT_VISUALIZATION */
  ) :
  TrackingAlgorithm(handle
  #ifdef SGT_VISUALIZATION
    , target_pub, steering_pose_pub
  #endif /* SGT_VISUALIZATION */
  )
{
}

void PurePursuit::update(const PathTrackingMsg &msg, sgtdv_msgs::ControlPtr &control_msg)
{    
  computeRearWheelPos(msg.car_pose);
  computeLookAheadDist(msg.car_vel->speed);
  const Eigen::Vector2f target_point = findTargetPoint(msg.trajectory);
  control_msg->steering_angle = computeSteeringCommand(msg, target_point); 
  control_msg->speed = computeSpeedCommand(msg.car_vel->speed, control_msg->speed);
}

void PurePursuit::computeRearWheelPos(const sgtdv_msgs::CarPose::ConstPtr &car_pose)
{
  const Eigen::Vector2f pos(car_pose->position.x, car_pose->position.y);
  rear_wheels_pos_ = pos - Eigen::Vector2f(cosf(car_pose->yaw) * params_.rear_wheels_offset, 
                                          sinf(car_pose->yaw)  * params_.rear_wheels_offset);
#ifdef SGT_VISUALIZATION
  visualizePoint(rear_wheels_pos_, 2, "rear wheels" , Eigen::Vector3f(0.0, 0.0, 1.0));
  const Eigen::Vector2f frontWheelsPos = 
    pos + Eigen::Vector2f(cosf(car_pose->yaw) * params_.front_wheels_offset,
                          sinf(car_pose->yaw) * params_.front_wheels_offset);
  visualizePoint(frontWheelsPos, 3, "front wheels" , Eigen::Vector3f(0.0, 0.0, 1.0));
#endif /* SGT_VISUALIZATION */
}

void PurePursuit::computeLookAheadDist(const float speed)
{
  const float lookahead_dist = params_.steering_k * speed;
  if(lookahead_dist < params_.lookahead_dist_min)
  {
    lookahead_dist_ = params_.lookahead_dist_min;
  }
  else if(lookahead_dist > params_.lookahead_dist_max)
  {
    lookahead_dist_ = params_.lookahead_dist_max;
  } 
  else
  {
    lookahead_dist_ = lookahead_dist;
  }
}

Eigen::Vector2f PurePursuit::findTargetPoint(const sgtdv_msgs::Point2DArr::ConstPtr &trajectory) const
{
  const auto size = trajectory->points.size();
  static Eigen::Vector2f target_point, next_point;

  const auto closest_idx = findClosestPointIdx(trajectory, rear_wheels_pos_);

  int offset(0), next_idx;
  target_point << trajectory->points[closest_idx].x,
                  trajectory->points[closest_idx].y;
  
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
    next_point << trajectory->points[next_idx].x,
                  trajectory->points[next_idx].y;

    if((rear_wheels_pos_ - next_point).norm() < lookahead_dist_)
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
    
    const auto bearing_vector = target_point - rear_wheels_pos_;
    const auto d = bearing_vector.norm();
    const auto theta = std::atan2(bearing_vector(1), bearing_vector(0));
    const auto gamma = M_PI - slope_angle + theta;

    const auto x = 
      d * cos(gamma) + std::sqrt(std::pow(d,2) * std::pow(cos(gamma),2) - std::pow(d,2) + std::pow(lookahead_dist_,2));

    target_point(0) += cos(slope_angle) * x,
    target_point(1) += sin(slope_angle) * x;
  }
#ifdef SGT_VISUALIZATION
  visualizePoint(target_point, 0, "target point", Eigen::Vector3f(1.0, 0.0, 0.0));
  visualizePoint(Eigen::Vector2f(trajectory->points[closest_idx].x, trajectory->points[closest_idx].y), 1,
    "closest point", Eigen::Vector3f(1.0, 1.0, 0.0));
#endif /* SGT_VISUALIZATION */

  return target_point;
}

float PurePursuit::computeSteeringCommand(const PathTrackingMsg &msg,
                                          const Eigen::Ref<const Eigen::Vector2f>& target_point)
{
  static float steering_angle = 0.0;
  const double theta = msg.car_pose->yaw;
  const double alpha 
    = std::atan2((target_point[1] - msg.car_pose->position.y), (target_point[0] - msg.car_pose->position.x)) - theta;
  steering_angle = static_cast<float>(std::atan2(2*std::sin(alpha)*params_.car_length,lookahead_dist_));

  // saturation
  steering_angle = std::max(params_.steering_min, std::min(params_.steering_max, steering_angle));

#ifdef SGT_VISUALIZATION
  visualizeSteering();
#endif /* SGT_VISUALIZATION */

  return steering_angle;
}
