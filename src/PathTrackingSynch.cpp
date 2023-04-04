/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/


#include "../include/PathTrackingSynch.h"

PathTrackingSynch::PathTrackingSynch(const ros::NodeHandle &handle) :
      m_pathTracking(handle)
    , m_trajectoryReady(false)
    , m_poseReady(false)
    , m_velocityReady(false)
{
    
}

void PathTrackingSynch::DoPlannedTrajectory(const sgtdv_msgs::Point2DArr::ConstPtr &msg)
{
    m_pathTrackingMsg.trajectory = msg;
    m_trajectoryReady = true;
}

void PathTrackingSynch::DoPoseEstimate(const sgtdv_msgs::CarPose::ConstPtr &msg)
{
    m_pathTrackingMsg.carPose = msg;
    m_poseReady = true;
}

void PathTrackingSynch::DoVelocityEstimate(const sgtdv_msgs::CarVel::ConstPtr &msg)
{
    m_pathTrackingMsg.carVel = msg;
    m_velocityReady = true;
}

void PathTrackingSynch::StopCallback(const std_msgs::Empty::ConstPtr &msg)
{
    m_pathTracking.StopVehicle();
}
void PathTrackingSynch::StartCallback(const std_msgs::Empty::ConstPtr &msg)
{
    m_pathTracking.StartVehicle();
}

void PathTrackingSynch::Do()
{
    while (ros::ok())
    {        
        ros::spinOnce();

        if(!m_trajectoryReady || !m_poseReady || !m_velocityReady) continue;
        
        auto start = std::chrono::steady_clock::now();

        m_pathTracking.Do(m_pathTrackingMsg);

        auto finish = std::chrono::steady_clock::now();
        auto timePerFrame = std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count() / 1000.f;
        float timeDiff = TIME_PER_FRAME - timePerFrame;

        if (timeDiff > 0.f)
        {
            sleep(timeDiff);
        }
        else
        {
            //defenzivne programovanie ftw
        } 
    }
}