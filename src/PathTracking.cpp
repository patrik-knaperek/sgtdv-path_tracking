/*****************************************************/
//Organization: Stuba Green Team
//Authors: Tereza Ábelová, Juraj Krasňanský
/*****************************************************/


#include "../include/PathTracking.h"

PathTracking::PathTracking(ros::NodeHandle &handle)
{
    //m_algorithm = new Stanley(handle); // malfunctioning, needs fix
    m_algorithm = new PurePursuit(handle);
    LoadParams(handle);
}

PathTracking::~PathTracking()
{

}

void PathTracking::LoadParams(ros::NodeHandle &handle)
{
    Params params;
    // load vehicle parameters
    std::cout << "LOADING PARAMETERS" << std::endl;
    if(!handle.getParam("/vehicle/car_length", params.carLength))
        ROS_ERROR("Failed to get parameter \"/vehicle/car_length\" from server\n");
    if(!handle.getParam("/vehicle/rear_wheels_offset", params.rearWheelsOffset))
        ROS_ERROR("Failed to get parameter \"/vehicle/rear_wheels_offset\" from server\n");
    if(!handle.getParam("/vehicle/front_wheels_offset", params.frontWheelsOffset))
        ROS_ERROR("Failed to get parameter \"/vehicle/front_wheels_offset\" from server\n");
    
    // load controller parameters
    if(!handle.getParam("/controller/speed/p", params.speedP))
        ROS_ERROR("Failed to get parameter \"/controller/speed/p\" from server\n");
    if(!handle.getParam("controller/speed/i", params.speedI))
        ROS_ERROR("Failed to get parameter \"/controller/speed/i\" from server\n");
    if(!handle.getParam("/controller/speed/min", params.speedMin))
        ROS_ERROR("Failed to get parameter \"/controller/speed/min\" from server\n");
    if(!handle.getParam("/controller/speed/max", params.speedMax))
        ROS_ERROR("Failed to get parameter \"/controller/speed/max\" from server\n");
    if(!handle.getParam("/controller/speed/ref_speed", params.refSpeed))
        ROS_ERROR("Failed to get parameter \"/controller/speed/ref_speed\" from server\n");
    if(!handle.getParam("/controller/speed/speed_raise_rate", params.speedRaiseRate))
        ROS_ERROR("Failed to get parameter \"/controller/speed/speed_raise_rate\" from server\n");
    if(!handle.getParam("/controller/steering/k", params.steeringK))
        ROS_ERROR("Failed to get parameter \"/controller/steering/k\" from server\n");
    if(!handle.getParam("/controller/steering/min", params.steeringMin))
        ROS_ERROR("Failed to get parameter \"/controller/steering/min\" from server\n");
    if(!handle.getParam("/controller/steering/max", params.steeringMax))
        ROS_ERROR("Failed to get parameter \"/controller/steering/max\" from server\n");
    if(!handle.getParam("/controller/steering/lookahead_dist_min", params.lookAheadDistMin))
        ROS_ERROR("Failed to get parameter \"/controller/steering/lookahead_dist_min\" from server\n");
    if(!handle.getParam("/controller/steering/lookahead_dist_max", params.lookAheadDistMax))
        ROS_ERROR("Failed to get parameter \"/controller/steering/lookahead_dist_max\" from server\n");
    m_algorithm->SetParams(params);
}

void PathTracking::SetPublishers(ros::Publisher cmdPub, ros::Publisher targetPub)
{
    m_publisher = cmdPub;
    m_algorithm->SetPublisher(targetPub);
}

/*void PathTracking::FreshTrajectory()
{
    m_algorithm->FreshTrajectory();
}*/

void PathTracking::Do(const PathTrackingMsg &msg)
{
    sgtdv_msgs::ControlPtr controlMsg(new sgtdv_msgs::Control);
    HandleAlgorithmResult(controlMsg, m_algorithm->Do(msg));

    m_publisher.publish(controlMsg);
}

void PathTracking::HandleAlgorithmResult(sgtdv_msgs::ControlPtr &msg, const Control &result)
{
    msg->speed = result.speed;
    msg->steeringAngle = result.steeringAngle;
}