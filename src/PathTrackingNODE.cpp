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

    ros::Subscriber trajectorySub = handle.subscribe("pathplanning_trajectory", 1, &PathTrackingSynch::DoPlannedTrajectory, &synchObj);
    ros::Subscriber actualStateSub = handle.subscribe("pose_estimate", 1, &PathTrackingSynch::DoActualState, &synchObj);
    //TODO add names of topics to subscribe to
    //ros::Subscriber approxStateSub = handle.subscrube("imu-topic", 1, &PathTrackingSynch::DoApproxState, &synchObj);
    //Do();

    return 0;
}