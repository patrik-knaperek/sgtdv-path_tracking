/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/

/* SGT-DV */
#include <sgtdv_msgs/Point2DArr.h>
#include <sgtdv_msgs/CarPose.h>
#include <sgtdv_msgs/CarVel.h>
#include <sgtdv_msgs/Trajectory.h>

#include "path_tracking.h"

/* ROS */
#include <ros/ros.h>
#include <std_srvs/Empty.h>

#ifdef SGT_VISUALIZATION
  #include <visualization_msgs/MarkerArray.h>
#endif

class PathTrackingROS
{
public:
  explicit PathTrackingROS(ros::NodeHandle& handle);
  ~PathTrackingROS() = default;
  
  void update();

private:
  PathTracking::Params loadParams(void);

  void trajectoryCallback(const sgtdv_msgs::Trajectory::ConstPtr &msg);
  void poseCallback(const sgtdv_msgs::CarPose::ConstPtr &msg);
  void velocityCallback(const sgtdv_msgs::CarVel::ConstPtr &msg);
  bool stopCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool startCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  #ifdef SGT_VISUALIZATION
  void initPurePursuitMarkers(void);
  void initMarker(const int point_id, const std::string& ns, const Eigen::Vector3f color);
  void visualizePoint(const Eigen::Vector2f& point, const int point_id);
#endif /* SGT_VISUALIZATION */

  ros::NodeHandle handle_;

  PathTracking path_tracking_obj_;
  sgtdv_msgs::PathTrackingMsg path_tracking_msg_;

  ros::Publisher cmd_pub_;
#ifdef SGT_VISUALIZATION
  visualization_msgs::MarkerArray pure_pursuit_vis_msg_;
  ros::Publisher pure_pursuit_vis_pub_;
  ros::Publisher steering_vis_pub_;
#endif /* SGT_VISUALIZATION */
#ifdef SGT_DEBUG_STATE
  ros::Publisher debug_vis_pub_;
#endif /* SGT_DEBUG_STATE */

  ros::Subscriber trajectory_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber velocity_sub_;
  ros::ServiceServer stop_server_;
  ros::ServiceServer start_server_;

  bool trajectory_ready_ = false;
  bool pose_ready_ = false;
  bool velocity_ready_ = false;
};
