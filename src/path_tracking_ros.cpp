/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/

/* Header */
#include "path_tracking_ros.h"

/* ROS */
#include <geometry_msgs/PoseStamped.h>

/* SGT-DV */
#ifdef SGT_DEBUG_STATE
  #include <sgtdv_msgs/DebugState.h>
#endif

PathTrackingROS::PathTrackingROS(ros::NodeHandle& handle)
  /* ROS interface init */
    : handle_(handle)
    , cmd_pub_(handle.advertise<sgtdv_msgs::Control>("path_tracking/cmd", 1))
  #ifdef SGT_VISUALIZATION
    , pure_pursuit_vis_pub_(handle.advertise<visualization_msgs::MarkerArray>("path_tracking/visualize/pure_pursuit",4))
    , steering_vis_pub_(handle.advertise<geometry_msgs::PoseStamped>("path_tracking/visualize/steering", 1))
  #endif /* SGT_VISUALIZATION */
  #ifdef SGT_DEBUG_STATE
    , debug_vis_pub_(handle.advertise<sgtdv_msgs::DebugState>("path_tracking/debug_state", 2)
  #endif /* SGT_DEBUG_STATE */
    )
    , trajectory_sub_(handle.subscribe("path_planning/trajectory", 1, &PathTrackingROS::trajectoryCallback, this))
    , pose_sub_(handle.subscribe("odometry/pose", 1, &PathTrackingROS::poseCallback, this))
    // , pose_sub_(handle.subscribe("slam/pose", 1, &PathTrackingROS::poseCallback, this))
    , velocity_sub_(handle.subscribe("odometry/velocity", 1, &PathTrackingROS::velocityCallback, this))
    , stop_server_(handle.advertiseService("path_tracking/stop", &PathTrackingROS::stopCallback, this))
    , start_server_(handle.advertiseService("path_tracking/start", &PathTrackingROS::startCallback, this))
    , set_speed_server_(handle.advertiseService("path_tracking/set_speed", &PathTrackingROS::setSpeedCallback, this))
{
  loadParams();

#ifdef SGT_VISUALIZATION
  initPurePursuitMarkers();
#endif
}

void PathTrackingROS::loadParams(void)
{
  ROS_INFO("LOADING PARAMETERS");
  PathTracking::Params params;
  
  /* load vehicle parameters */
  Utils::loadParam(handle_, "/vehicle/car_length", &params.car_length);
  Utils::loadParam(handle_, "/vehicle/rear_wheels_offset", &params.rear_wheels_offset);
  Utils::loadParam(handle_, "/vehicle/front_wheels_offset", &params.front_wheels_offset);
  
  /* load PID controller parameters */
  Utils::loadParam(handle_, "/controller/speed/p", &params.speed_p);
  Utils::loadParam(handle_, "controller/speed/i", &params.speed_i);
  Utils::loadParam(handle_, "/controller/speed/min", &params.speed.min);
  Utils::loadParam(handle_, "/controller/speed/max", &params.speed.max);
  // Utils::loadParam(handle_, "/controller/speed/ref_speed", &params.refSpeed);
  Utils::loadParam(handle_, "/controller/speed/speed_raise_rate", &params.speed_raise_rate);
  Utils::loadParam(handle_, "/controller/steering/k", &params.steering_k);
  Utils::loadParam(handle_, "/controller/steering/smooth", &params.steering_smooth);
  Utils::loadParam(handle_, "/controller/steering/min", &params.steering.min);
  Utils::loadParam(handle_, "/controller/steering/max", &params.steering.max);
  Utils::loadParam(handle_, "/controller/steering/lookahead_dist_min", &params.lookahead_dist.min);
  Utils::loadParam(handle_, "/controller/steering/lookahead_dist_max", &params.lookahead_dist.max);

  Utils::loadParam(handle_, "/track_loop", true, &params.track_loop);
  path_tracking_obj_.setParams(params);
}


void PathTrackingROS::trajectoryCallback(const sgtdv_msgs::Point2DArr::ConstPtr &msg)
{
  path_tracking_msg_.trajectory = *msg;
  trajectory_ready_ = true;
}

void PathTrackingROS::poseCallback(const sgtdv_msgs::CarPose::ConstPtr &msg)
{
  path_tracking_msg_.car_pose = *msg;
  pose_ready_ = true;
}

void PathTrackingROS::velocityCallback(const sgtdv_msgs::CarVel::ConstPtr &msg)
{
  path_tracking_msg_.car_vel = *msg;
  velocity_ready_ = true;
}

bool PathTrackingROS::stopCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  if(!stopped_)
  {
    stopped_ = true;
    ROS_INFO("STOPPING VEHICLE");
  }
  return true;
}
bool PathTrackingROS::startCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  loadParams();
  path_tracking_obj_.resetIntegral();
  stopped_ = false;

  ROS_INFO("STARTING VEHICLE");

  return true;
}

bool PathTrackingROS::setSpeedCallback(sgtdv_msgs::Float32Srv::Request &req, sgtdv_msgs::Float32Srv::Response &res)
{
  path_tracking_obj_.setRefSpeed(req.data);
  return 1;
}

void PathTrackingROS::update(void)
{ 
  if(trajectory_ready_ && pose_ready_ && velocity_ready_)
  {
  #ifdef SGT_DEBUG_STATE
    sgtdv_msgs::DebugState state;
    state.stamp = ros::Time::now();
    state.working_state = 1;
    debug_vis_pub_.publish(state);
  #endif // SGT_DEBUG_STATE

    static sgtdv_msgs::Control control_msg;

    if(stopped_)
    {
      control_msg.speed = 0.0;
      control_msg.steering_angle = 0.0;
    }
    else
    {
      path_tracking_obj_.update(path_tracking_msg_, &control_msg);

    #ifdef SGT_VISUALIZATION
      const auto pp_points = path_tracking_obj_.getPurePursuitPoints();
      visualizePoint(pp_points.first, 0); // target point
      visualizePoint(pp_points.second, 1);  // closest trajectory point

      const auto axle_pos = path_tracking_obj_.getAxlePositions();
      visualizePoint(axle_pos.first, 2);  // rear axle position
      visualizePoint(axle_pos.second, 3); // front axle position
      pure_pursuit_vis_pub_.publish(pure_pursuit_vis_msg_);
    #endif /* SGT_VISUALIZATION */
    }

    control_msg.stamp = ros::Time::now();
    cmd_pub_.publish(control_msg);
    
  #ifdef SGT_DEBUG_STATE
    state.stamp = ros::Time::now();
    state.working_state = 0;
    state.speed = control_msg.speed;
    state.angle = control_msg.steering_angle;
    debug_vis_pub_.publish(state);
  #endif // SGT_DEBUG_STATE
  }
}

#ifdef SGT_VISUALIZATION
void PathTrackingROS::initPurePursuitMarkers(void)
{
  pure_pursuit_vis_msg_.markers.reserve(4);
  initMarker(0, "target point", Eigen::Vector3f(1.0, 0.0, 0.0));
  initMarker(1, "closest point", Eigen::Vector3f(1.0, 1.0, 0.0));
  initMarker(2, "rear axle" , Eigen::Vector3f(0.0, 0.0, 1.0));
  initMarker(3, "front axle" , Eigen::Vector3f(0.0, 0.0, 1.0));
}

void PathTrackingROS::initMarker(const int point_id, const std::string& ns, const Eigen::Vector3f color)
{
  visualization_msgs::Marker marker;
  
  marker.color.r              = color(0);
  marker.color.g              = color(1);
  marker.color.b              = color(2);
  marker.color.a              = 1.0;
  marker.pose.orientation.w   = 1.0;
  marker.type                 = visualization_msgs::Marker::SPHERE;
  marker.action               = visualization_msgs::Marker::ADD;
  marker.id                   = point_id;
  marker.ns                   = ns;
  marker.scale.x              = 0.3;
  marker.scale.y              = 0.3;
  marker.scale.z              = 0.3;
  marker.header.frame_id      = "map";
  pure_pursuit_vis_msg_.markers.emplace_back(marker);
}
void PathTrackingROS::visualizePoint(const Eigen::Vector2f& point, const int point_id)
{
  pure_pursuit_vis_msg_.markers.at(point_id).pose.position.x      = point(0);
  pure_pursuit_vis_msg_.markers.at(point_id).pose.position.y      = point(1);
  pure_pursuit_vis_msg_.markers.at(point_id).header.stamp         = ros::Time::now();
}
#endif /* SGT_VISUALIZATION */
