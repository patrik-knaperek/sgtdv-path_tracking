/*****************************************************/
//Organization: Stuba Green Team
//Authors: Tereza Ábelová, Juraj Krasňanský
/*****************************************************/


#include "../include/PathTracking.h"

PathTracking::PathTracking(const ros::NodeHandle &handle) :
  m_handle(handle)
, m_algorithm(new PurePursuit(handle))
, m_stopped(true)
{
    //m_algorithm = new Stanley(handle); // malfunctioning, needs fix
    LoadParams(handle);
}

void PathTracking::LoadParams(const ros::NodeHandle &handle) const
{
    ROS_INFO("LOADING PARAMETERS");
    TrackingAlgorithm::Params params;
    
    /* load vehicle parameters */
    Utils::loadParam(handle, "/vehicle/car_length", &params.carLength);
    Utils::loadParam(handle, "/vehicle/rear_wheels_offset", &params.rearWheelsOffset);
    Utils::loadParam(handle, "/vehicle/front_wheels_offset", &params.frontWheelsOffset);
    
    /* load PID controller parameters */
    Utils::loadParam(handle, "/controller/speed/p", &params.speedP);
    Utils::loadParam(handle, "controller/speed/i", &params.speedI);
    Utils::loadParam(handle, "/controller/speed/min", &params.speedMin);
    Utils::loadParam(handle, "/controller/speed/max", &params.speedMax);
    // Utils::loadParam(handle, "/controller/speed/ref_speed", &params.refSpeed);
    Utils::loadParam(handle, "/controller/speed/speed_raise_rate", &params.speedRaiseRate);
    Utils::loadParam(handle, "/controller/steering/k", &params.steeringK);
    Utils::loadParam(handle, "/controller/steering/min", &params.steeringMin);
    Utils::loadParam(handle, "/controller/steering/max", &params.steeringMax);
    Utils::loadParam(handle, "/controller/steering/lookahead_dist_min", &params.lookAheadDistMin);
    Utils::loadParam(handle, "/controller/steering/lookahead_dist_max", &params.lookAheadDistMax);

    Utils::loadParam(handle, "/track_loop", true, &params.trackLoop);
    m_algorithm->SetParams(params);
    
}

void PathTracking::StopVehicle()
{
    if (!m_stopped)
    {
	    m_stopped = true;
    	ROS_INFO("STOPPING VEHICLE");
    }
}

void PathTracking::StartVehicle()
{
    LoadParams(m_handle);
    m_algorithm->ResetIntegral();
    m_stopped = false;
    ROS_INFO("STARTING VEHICLE");
}

void PathTracking::Do(const PathTrackingMsg &msg)
{
    static boost::shared_ptr<sgtdv_msgs::Control> controlMsg = boost::make_shared<sgtdv_msgs::Control>();
    if (m_stopped)
    {
        controlMsg->speed = 0.0;
        controlMsg->steeringAngle = 0.0;
    } else
    {
        m_algorithm->Do(msg, controlMsg);
    }

    m_cmdPublisher.publish(controlMsg);
}