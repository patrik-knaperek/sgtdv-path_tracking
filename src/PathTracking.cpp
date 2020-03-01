/*****************************************************/
//Organization: Stuba Green Team
//Authors: Tereza Ábelová, Juraj Krasňanský
/*****************************************************/


#include "../include/PathTracking.h"

PathTracking::PathTracking()
{
    m_algorithm = new Stanley;
}

PathTracking::~PathTracking()
{

}

void PathTracking::SetPublisher(ros::Publisher publisher)
{
    m_publisher = publisher;
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