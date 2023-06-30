/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/

#include "../include/PathTrackingSynch.h"

int main (int argc, char** argv)
{
    ros::init(argc, argv, "pathTracking");
    ros::NodeHandle handle;
    PathTrackingSynch synchObj(handle);

    ros::Publisher commandPublisher = handle.advertise<sgtdv_msgs::Control>("pathtracking_commands", 1);
    ros::Publisher targetPublisher = handle.advertise<visualization_msgs::Marker>("pathtracking_marker",1);
    ros::Publisher steeringPosePublisher = handle.advertise<geometry_msgs::PoseStamped>("steering_angle_pathtracking", 1);

    synchObj.SetCmdPublisher(commandPublisher);
    synchObj.SetVisualizationPublishers(targetPublisher, steeringPosePublisher);    

    ros::Subscriber trajectorySub = handle.subscribe("pathplanning_trajectory", 1, &PathTrackingSynch::DoPlannedTrajectory, &synchObj);
    // ros::Subscriber poseSub = handle.subscribe("pose_estimate", 1, &PathTrackingSynch::DoPoseEstimate, &synchObj);
    ros::Subscriber poseSub = handle.subscribe("slam/pose", 1, &PathTrackingSynch::DoPoseEstimate, &synchObj);
    ros::Subscriber velocitySub = handle.subscribe("velocity_estimate", 1, &PathTrackingSynch::DoVelocityEstimate, &synchObj);
    ros::Subscriber stopSub = handle.subscribe("stop", 1, &PathTrackingSynch::StopCallback, &synchObj);
    ros::Subscriber startSub = handle.subscribe("start", 1, &PathTrackingSynch::StartCallback, &synchObj);

    ros::Rate looprate(FPS);
    while (ros::ok())
    {
        ros::spinOnce();
        synchObj.Do();
        looprate.sleep();
    }

    return 0;
}