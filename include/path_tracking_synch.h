/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/

/* ROS */
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>

/* SGT */
#include <sgtdv_msgs/Point2DArr.h>
#include <sgtdv_msgs/CarPose.h>
#include <sgtdv_msgs/CarVel.h>
#include <sgtdv_msgs/Float32Srv.h>
#include "../include/messages.h"
#include "../include/path_tracking.h"

class PathTrackingSynch
{
public:
  PathTrackingSynch(const ros::NodeHandle &handle);
  ~PathTrackingSynch() = default;

  void setCmdPublisher(const ros::Publisher &cmd_pub)
  {
    path_tracking_obj_.setCmdPublisher(cmd_pub);
  };        
#ifdef SGT_VISUALIZATION
  void setVisualizationPublishers(const ros::Publisher &target_pub, const ros::Publisher &steering_pose_pub)
  {
    path_tracking_obj_.setVisualizationPublishers(target_pub, steering_pose_pub);
  };
#endif /* SGT_VISUALIZATION */
#ifdef SGT_DEBUG_STATE
  void setVisDebugPublisher(ros::Publisher publisher) { path_tracking_obj_.SetVisDebugPublisher(publisher); };
#endif
  
  void doPlannedTrajectory(const sgtdv_msgs::Point2DArr::ConstPtr &msg);
  void doPoseEstimate(const sgtdv_msgs::CarPose::ConstPtr &msg);
  void doVelocityEstimate(const sgtdv_msgs::CarVel::ConstPtr &msg);
  bool stopCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool startCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool setSpeedCallback(sgtdv_msgs::Float32Srv::Request &req, sgtdv_msgs::Float32Srv::Response &res);
  void update();

private:
  PathTracking path_tracking_obj_;
  PathTrackingMsg path_tracking_msg_;

  bool trajectory_ready_;
  bool pose_ready_;
  bool velocity_ready_;
  sgtdv_msgs::CarPose last_pose_;

  // parameters
  float ref_speed_;
  float const_yaw_rate_;
};
