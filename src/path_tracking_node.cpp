/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/

#include "path_tracking_synch.h"

int main (int argc, char** argv)
{
  ros::init(argc, argv, "path_tracking");
  ros::NodeHandle handle;
  PathTrackingSynch synch_obj(handle);

  ros::Rate looprate(FPS);
  while(ros::ok())
  {
    ros::spinOnce();
    synch_obj.update();
    looprate.sleep();
  }

  return 0;
}
