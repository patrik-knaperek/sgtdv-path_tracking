/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/


#include "../include/PathTrackingSynch.h"

PathTrackingSynch::PathTrackingSynch(ros::NodeHandle &handle) :
    m_pathTracking(handle)
{
    
}

PathTrackingSynch::~PathTrackingSynch()
{

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

    static bool firstMessage = true;
    if (firstMessage)
    {
        this->SetLastPose(msg);
        firstMessage = false;
        return;
    }
    
    sgtdv_msgs::CarPosePtr poseDelta(new sgtdv_msgs::CarPose);
    poseDelta->position.x = std::abs(msg->position.x - m_lastPose.position.x);
    poseDelta->position.y = std::abs(msg->position.y - m_lastPose.position.y);
    poseDelta->yaw = msg->yaw - m_lastPose.yaw;
    double timeDelta = std::abs(msg->position.header.stamp.toSec() - m_lastPose.position.header.stamp.toSec());
    
    if (timeDelta)
    {
        VelocityEstimate(m_pathTrackingMsg, poseDelta, timeDelta);

        this->SetLastPose(msg);
    }
}

void PathTrackingSynch::SetLastPose(const sgtdv_msgs::CarPose::ConstPtr &msg)
{
    m_lastPose.position.x = msg->position.x;
    m_lastPose.position.y = msg->position.y;
    m_lastPose.position.header.stamp = msg->position.header.stamp;
    m_lastPose.yaw = msg->yaw;
}

void PathTrackingSynch::VelocityEstimate(PathTrackingMsg &msg, const sgtdv_msgs::CarPosePtr &poseDelta, double &timeDelta)
{
    
    const double yawRate = poseDelta->yaw / timeDelta;
    msg.yawRate = static_cast<float>(yawRate);
    
    if (poseDelta->position.x == 0 && poseDelta->position.y == 0)
    {
        msg.speed = 0;
    } else if (poseDelta->yaw == 0)
    {
        msg.speed = std::hypot(poseDelta->position.x, poseDelta->position.y) / timeDelta;
    } else
    {
        const double R = std::hypot(poseDelta->position.x, poseDelta->position.y) / (2 * std::sin(std::abs(poseDelta->yaw) / 2));
        msg.speed = static_cast<float>(std::abs(yawRate) * R);
    }
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