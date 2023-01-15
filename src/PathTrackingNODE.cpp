/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/


#include <ros/ros.h>
#include "../include/PathTrackingSynch.h"
#include <sgtdv_msgs/Control.h>
#include <visualization_msgs/Marker.h>

int main (int argc, char** argv)
{
    ros::init(argc, argv, "pathTracking");
    ros::NodeHandle handle;
    PathTrackingSynch synchObj(handle);

    ros::Publisher commandPublisher = handle.advertise<sgtdv_msgs::Control>("pathtracking_commands", 1);
    ros::Publisher targetPublisher = handle.advertise<visualization_msgs::Marker>("pathtracking_target",1);

    synchObj.SetPublishers(commandPublisher, targetPublisher);    

    ros::Subscriber trajectorySub = handle.subscribe("pathplanning_trajectory", 1, &PathTrackingSynch::DoPlannedTrajectory, &synchObj);
    ros::Subscriber actualStateSub = handle.subscribe("pose_estimate", 1, &PathTrackingSynch::DoPoseEstimate, &synchObj);

    synchObj.Do();

    return 0;
}