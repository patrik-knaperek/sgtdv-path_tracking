/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

#include "path_tracking_ros.h"

int main (int argc, char** argv)
{
  ros::init(argc, argv, "path_tracking");
  ros::NodeHandle handle;
  PathTrackingROS ros_obj(handle);

  ros::Rate looprate(FPS);
  while(ros::ok())
  {
    ros::spinOnce();
    ros_obj.update();
    looprate.sleep();
  }

  return 0;
}
