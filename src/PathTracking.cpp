/*****************************************************/
//Organization: Stuba Green Team
//Authors: Tereza Ábelová, Juraj Krasňanský
/*****************************************************/


#include "../include/PathTracking.h"

PathTracking::PathTracking(ros::NodeHandle &handle)
{
    //m_algorithm = new Stanley(handle);
    m_algorithm = new PurePursuit(handle);
}

PathTracking::~PathTracking()
{

}

void PathTracking::SetPublishers(ros::Publisher cmdPub, ros::Publisher targetPub)
{
    m_publisher = cmdPub;
    m_algorithm->SetPublisher(targetPub);
}

void PathTracking::FreshTrajectory()
{
    m_algorithm->FreshTrajectory();
}

void PathTracking::Do(const PathTrackingMsg &msg)
{
    sgtdv_msgs::ControlPtr controlMsg( new sgtdv_msgs::Control );
    HandleAlgorithmResult(controlMsg, m_algorithm->Do(msg));

    m_publisher.publish(controlMsg);
}

void PathTracking::HandleAlgorithmResult(sgtdv_msgs::ControlPtr &msg, const Control &result)
{
    msg->speed = result.speed;
    msg->steeringAngle = result.steeringAngle;
}