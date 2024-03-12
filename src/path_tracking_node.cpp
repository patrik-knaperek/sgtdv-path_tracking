/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/

#include "../include/path_tracking_synch.h"

int main (int argc, char** argv)
{
  ros::init(argc, argv, "path_tracking");
  ros::NodeHandle handle;
  PathTrackingSynch synch_obj(handle);

  ros::Publisher command_publisher = handle.advertise<sgtdv_msgs::Control>("pathtracking_commands", 1);
  synch_obj.setCmdPublisher(command_publisher);

#ifdef SGT_VISUALIZATION
  ros::Publisher target_publisher = handle.advertise<visualization_msgs::Marker>("pathtracking/visualize/target",4);
  ros::Publisher steering_pose_publisher 
    = handle.advertise<geometry_msgs::PoseStamped>("pathtracking/visualize/steering", 1);
  synch_obj.setVisualizationPublishers(target_publisher, steering_pose_publisher);
#endif /* SGT_VISUALIZATION */
#ifdef SGT_DEBUG_STATE
  ros::Publisher pathtracking_debug_state_publisher
    = handle.advertise<sgtdv_msgs::DebugState>("pathtracking_debug_state", 10);
  synch_obj.setVisDebugPublisher(pathtracking_debug_state_publisher);
#endif

  ros::Subscriber trajectory_sub
    = handle.subscribe("pathplanning_trajectory", 10, &PathTrackingSynch::doPlannedTrajectory, &synch_obj);
  ros::Subscriber pose_sub = handle.subscribe("pose_estimate", 10, &PathTrackingSynch::doPoseEstimate, &synch_obj);
  // ros::Subscriber pose_sub = handle.subscribe("slam/pose", 1, &PathTrackingSynch::doPoseEstimate, &synchObj);
  ros::Subscriber velocity_sub
    = handle.subscribe("velocity_estimate", 10, &PathTrackingSynch::doVelocityEstimate, &synch_obj);
  ros::ServiceServer stop_sub
    = handle.advertiseService("path_tracking/stop", &PathTrackingSynch::stopCallback, &synch_obj);
  ros::ServiceServer start_sub
    = handle.advertiseService("path_tracking/start", &PathTrackingSynch::startCallback, &synch_obj);
  ros::ServiceServer set_speed_server 
    = handle.advertiseService("path_tracking/set_speed", &PathTrackingSynch::setSpeedCallback, &synch_obj);

  ros::Rate looprate(FPS);
  while(ros::ok())
  {
    ros::spinOnce();
    synch_obj.update();
    looprate.sleep();
  }

  return 0;
}
