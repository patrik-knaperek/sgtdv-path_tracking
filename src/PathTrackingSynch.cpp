/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/


#include "../include/PathTrackingSynch.h"

PathTrackingSynch::PathTrackingSynch(ros::NodeHandle &handle) :
    m_pathTracking(handle)
{
    //m_pathTrackingMsg.speed = CONST_SPEED;
    //m_pathTrackingMsg.yawRate = CONST_YAW_RATE;
}

PathTrackingSynch::~PathTrackingSynch()
{

}

void PathTrackingSynch::SetParams(float constSpeed, float constYawRate)
{
                    m_constSpeed = constSpeed;
                    m_constYawRate = constYawRate;

                    m_pathTrackingMsg.speed = m_constSpeed;
                    m_pathTrackingMsg.yawRate = m_constYawRate;
}

void PathTrackingSynch::SetPublishers(ros::Publisher cmdPub, ros::Publisher targetPub)
{
    m_pathTracking.SetPublishers(cmdPub, targetPub);
}

void PathTrackingSynch::DoPlannedTrajectory(const sgtdv_msgs::Point2DArr::ConstPtr &msg)
{
    m_pathTrackingMsg.trajectory = msg;
    m_pathTracking.FreshTrajectory();
    m_trajectoryReady = true;
}

void PathTrackingSynch::DoPoseEstimate(const sgtdv_msgs::CarPose::ConstPtr &msg)
{
    m_pathTrackingMsg.carPose = msg;
}

void PathTrackingSynch::Do()
{
    while (ros::ok())
    {        
        ros::spinOnce();

        if(!m_trajectoryReady) continue;
        
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