/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/

/* ROS */
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

/* SGT-DV */
#include <sgtdv_msgs/DebugState.h>

/* Header */
#include "path_tracking_synch.h"

PathTrackingSynch::PathTrackingSynch(ros::NodeHandle& handle) :
  /* ROS interface init */
  path_tracking_obj_(handle,
    handle.advertise<sgtdv_msgs::Control>("pathtracking_commands", 1)
  #ifdef SGT_VISUALIZATION
    , handle.advertise<visualization_msgs::Marker>("pathtracking/visualize/pure_pursuit",4)
    , handle.advertise<geometry_msgs::PoseStamped>("pathtracking/visualize/steering", 1)
  #endif /* SGT_VISUALIZATION */
  #ifdef SGT_DEBUG_STATE
    , handle.advertise<sgtdv_msgs::DebugState>("pathtracking_debug_state", 2)
  #endif /* SGT_DEBUG_STATE */
    ),
    trajectory_sub_(handle.subscribe("pathplanning_trajectory", 1, &PathTrackingSynch::trajectoryCallback, this)),
    pose_sub_(handle.subscribe("pose_estimate", 1, &PathTrackingSynch::poseCallback, this)),
  // pose_sub_(handle.subscribe("slam/pose", 1, &PathTrackingSynch::poseCallback, this)),
  velocity_sub_(handle.subscribe("velocity_estimate", 1, &PathTrackingSynch::velocityCallback, this)),
  stop_sub_(handle.advertiseService("path_tracking/stop", &PathTrackingSynch::stopCallback, this)),
  start_sub_(handle.advertiseService("path_tracking/start", &PathTrackingSynch::startCallback, this)),
  set_speed_server_(handle.advertiseService("path_tracking/set_speed", &PathTrackingSynch::setSpeedCallback, this)),

  trajectory_ready_(false),
  pose_ready_(false),
  velocity_ready_(false)
{
}

void PathTrackingSynch::trajectoryCallback(const sgtdv_msgs::Point2DArr::ConstPtr &msg)
{
  path_tracking_msg_.trajectory = msg;
  trajectory_ready_ = true;
}

void PathTrackingSynch::poseCallback(const sgtdv_msgs::CarPose::ConstPtr &msg)
{
  path_tracking_msg_.car_pose = msg;
  pose_ready_ = true;
}

void PathTrackingSynch::velocityCallback(const sgtdv_msgs::CarVel::ConstPtr &msg)
{
  path_tracking_msg_.car_vel = msg;
  velocity_ready_ = true;
}

bool PathTrackingSynch::stopCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  path_tracking_obj_.stopVehicle();
  return true;
}
bool PathTrackingSynch::startCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  path_tracking_obj_.startVehicle();
  return true;
}

bool PathTrackingSynch::setSpeedCallback(sgtdv_msgs::Float32Srv::Request &req, sgtdv_msgs::Float32Srv::Response &res)
{
  path_tracking_obj_.setRefSpeed(req.data);
  return 1;
}

void PathTrackingSynch::update()
{
  
  if(trajectory_ready_ && pose_ready_ && velocity_ready_)
  {
    path_tracking_obj_.update(path_tracking_msg_);
  }
}
