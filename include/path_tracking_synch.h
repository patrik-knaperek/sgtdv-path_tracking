/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/

/* ROS */
#include <ros/ros.h>
#include <std_srvs/Empty.h>

/* SGT-DV */
#include <sgtdv_msgs/Point2DArr.h>
#include <sgtdv_msgs/CarPose.h>
#include <sgtdv_msgs/CarVel.h>
#include <sgtdv_msgs/Float32Srv.h>
#include "messages.h"
#include "path_tracking.h"

class PathTrackingSynch
{
public:
  explicit PathTrackingSynch(ros::NodeHandle& handle);
  ~PathTrackingSynch() = default;
  
  void update();

private:
  void trajectoryCallback(const sgtdv_msgs::Point2DArr::ConstPtr &msg);
  void poseCallback(const sgtdv_msgs::CarPose::ConstPtr &msg);
  void velocityCallback(const sgtdv_msgs::CarVel::ConstPtr &msg);
  bool stopCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool startCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool setSpeedCallback(sgtdv_msgs::Float32Srv::Request &req, sgtdv_msgs::Float32Srv::Response &res);

  PathTracking path_tracking_obj_;
  PathTrackingMsg path_tracking_msg_;

  ros::Subscriber trajectory_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber velocity_sub_;
  ros::ServiceServer stop_server_;
  ros::ServiceServer start_server_;
  ros::ServiceServer set_speed_server_;

  bool trajectory_ready_;
  bool pose_ready_;
  bool velocity_ready_;
};
