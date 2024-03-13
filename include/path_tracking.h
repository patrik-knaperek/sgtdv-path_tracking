/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/

#pragma once

/* C++ */
#include <chrono>

/* ROS */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

/* SGT */
#include <sgtdv_msgs/PathTrackingMsg.h>
#include <sgtdv_msgs/Control.h>
#include "../include/messages.h"
#include "../include/tracking_algorithms.h"
#include <sgtdv_msgs/DebugState.h>
#include "../../SGT_Utils.h"

class PathTracking
{
public:
  PathTracking(const ros::NodeHandle &handle, const ros::Publisher& cmd_pub
  #ifdef SGT_VISUALIZATION
    , const ros::Publisher& target_pub
    , const ros::Publisher& steering_pose_pub
  #endif /* SGT_VISUALIZATION */
  #ifdef SGT_DEBUG_STATE
    , const ros::Publisher& vis_debug_pub
  #endif /* SGT_DEBUG_STATE */
  );
  ~PathTracking() = default;

  void loadParams(const ros::NodeHandle &handle) const;

  void update(const PathTrackingMsg &msg);
  void stopVehicle();
  void startVehicle();
  void setRefSpeed(const float ref_speed)
  {
    algorithm_->setRefSpeed(ref_speed);
  }
private:
  ros::NodeHandle handle_;
  TrackingAlgorithm *algorithm_;
  ros::Publisher cmd_pub_;
  bool stopped_;

#ifdef SGT_DEBUG_STATE
  ros::Publisher vis_debug_pub_;
#endif
};
