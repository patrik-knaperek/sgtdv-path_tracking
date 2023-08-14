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
    synchObj.SetCmdPublisher(commandPublisher);

#ifdef SGT_VISUALIZATION
    ros::Publisher targetPublisher = handle.advertise<visualization_msgs::Marker>("pathtracking/visualize/target",1);
    ros::Publisher steeringPosePublisher = handle.advertise<geometry_msgs::PoseStamped>("pathtracking/visualize/steering", 1);
    synchObj.SetVisualizationPublishers(targetPublisher, steeringPosePublisher);
#endif /* SGT_VISUALIZATION */

    ros::Subscriber trajectorySub = handle.subscribe("pathplanning_trajectory", 1, &PathTrackingSynch::DoPlannedTrajectory, &synchObj);
    ros::Subscriber poseSub = handle.subscribe("pose_estimate", 1, &PathTrackingSynch::DoPoseEstimate, &synchObj);
    // ros::Subscriber poseSub = handle.subscribe("slam/pose", 1, &PathTrackingSynch::DoPoseEstimate, &synchObj);
    ros::Subscriber velocitySub = handle.subscribe("velocity_estimate", 1, &PathTrackingSynch::DoVelocityEstimate, &synchObj);
    ros::ServiceServer stopSub = handle.advertiseService("pathTracking/stop", &PathTrackingSynch::StopCallback, &synchObj);
    ros::ServiceServer startSub = handle.advertiseService("pathTracking/start", &PathTrackingSynch::StartCallback, &synchObj);
    ros::ServiceServer setSpeedServer 
        = handle.advertiseService("pathTracking/set_speed", &PathTrackingSynch::SetSpeedCallback, &synchObj);

    ros::Rate looprate(FPS);
    while (ros::ok())
    {
        ros::spinOnce();
        synchObj.Do();
        looprate.sleep();
    }

    return 0;
}
