/*****************************************************/
//Organization: Stuba Green Team
//Authors: Tereza Ábelová, Juraj Krasňanský
/*****************************************************/


#include "../include/PathTracking.h"

PathTracking::PathTracking(const ros::NodeHandle &handle) :
  m_algorithm(new PurePursuit(handle))
, m_stopped(true)
{
    //m_algorithm = new Stanley(handle); // malfunctioning, needs fix
    LoadParams(handle);
}

void PathTracking::LoadParams(const ros::NodeHandle &handle) const
{
    ROS_INFO("LOADING PARAMETERS");
    Params params;
    // load vehicle parameters 
    params.carLength = GetParam(handle, "/vehicle/car_length");
    params.rearWheelsOffset = GetParam(handle, "/vehicle/rear_wheels_offset");
    params.frontWheelsOffset = GetParam(handle, "/vehicle/front_wheels_offset");
    
    // load controller parameters
    params.speedP = GetParam(handle, "/controller/speed/p");
    params.speedI = GetParam(handle, "controller/speed/i");
    params.speedMin = GetParam(handle, "/controller/speed/min");
    params.speedMax = GetParam(handle, "/controller/speed/max");
    params.refSpeed = GetParam(handle, "/controller/speed/ref_speed");
    params.speedRaiseRate = GetParam(handle, "/controller/speed/speed_raise_rate");
    params.steeringK = GetParam(handle, "/controller/steering/k");
    params.steeringMin = GetParam(handle, "/controller/steering/min");
    params.steeringMax = GetParam(handle, "/controller/steering/max");
    params.lookAheadDistMin = GetParam(handle, "/controller/steering/lookahead_dist_min");
    params.lookAheadDistMax = GetParam(handle, "/controller/steering/lookahead_dist_max");
    
    //params.trackLoop = static_cast<bool>(GetParam(handle, "/track_loop"));
    params.trackLoop = true;
    
    m_algorithm->SetParams(params);
}

float PathTracking::GetParam(const ros::NodeHandle &handle, const std::string &name) const
{
    float storage;
    if(!handle.getParam(name, storage))
        ROS_ERROR("Failed to get parameter \"%s\" from server\n", name.data());
    return storage;
}

void PathTracking::StopVehicle()
{
    m_stopped = true;
    ROS_INFO("STOPPING VEHICLE");
}

void PathTracking::StartVehicle()
{
    m_stopped = false;
    ROS_INFO("STARTING VEHICLE");
}

void PathTracking::Do(const PathTrackingMsg &msg)
{
    //sgtdv_msgs::ControlPtr controlMsg(new sgtdv_msgs::Control);
    boost::shared_ptr<sgtdv_msgs::Control> controlMsg = boost::make_shared<sgtdv_msgs::Control>();
    if (m_stopped)
    {
        controlMsg->speed = 0.0;
        controlMsg->steeringAngle = 0.0;
    } else
    {
        HandleAlgorithmResult(controlMsg, m_algorithm->Do(msg));
    }

    m_cmdPublisher.publish(controlMsg);
}

void PathTracking::HandleAlgorithmResult(sgtdv_msgs::ControlPtr &msg, const Control &result)
{
    msg->speed = result.speed;
    msg->steeringAngle = result.steeringAngle;
}