#include <ros/ros.h>
#include "../include/PathTrackingSynch.h"
#include <sgtdv_msgs/Control.h>

int main (int argc, char** argv)
{
    PathTrackingSynch synchObj;

    ros::init(argc, argv, "pathTracking");
    ros::NodeHandle handle;

    ros::Publisher publisher = handle.advertise<sgtdv_msgs::Control>("pathtracking_commands", 1);

    synchObj.SetPublisher(publisher);    

    //TODO add names of topics to subscribe to
    //ros::Subscriber mapSub = handle.subscribe("slam_map", 1, &PathTrackingSynch::Do, &synchObj);
    //ros::Subscriber poseSub = handle.subscribe("slam_pose", 1, &PathTrackingSynch::UpdatePose, &synchObj);
    
    ros::spin();

    return 0;
}