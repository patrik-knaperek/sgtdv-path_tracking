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
    GetParam(handle, "/vehicle/car_length", &params.carLength);
    GetParam(handle, "/vehicle/rear_wheels_offset", &params.rearWheelsOffset);
    GetParam(handle, "/vehicle/front_wheels_offset", &params.frontWheelsOffset);
    
    // load controller parameters
    GetParam(handle, "/controller/speed/p", &params.speedP);
    GetParam(handle, "controller/speed/i", &params.speedI);
    GetParam(handle, "/controller/speed/min", &params.speedMin);
    GetParam(handle, "/controller/speed/max", &params.speedMax);
    // GetParam(handle, "/controller/speed/ref_speed", &params.refSpeed);
    GetParam(handle, "/controller/speed/speed_raise_rate", &params.speedRaiseRate);
    GetParam(handle, "/controller/steering/k", &params.steeringK);
    GetParam(handle, "/controller/steering/min", &params.steeringMin);
    GetParam(handle, "/controller/steering/max", &params.steeringMax);
    GetParam(handle, "/controller/steering/lookahead_dist_min", &params.lookAheadDistMin);
    GetParam(handle, "/controller/steering/lookahead_dist_max", &params.lookAheadDistMax);
    
    GetParam(handle, "/track_loop", true, &params.trackLoop);
    m_algorithm->SetParams(params);
    
}

template<typename T>
void PathTracking::GetParam(const ros::NodeHandle &handle, const std::string &name, T* storage) const
{
    if (!handle.getParam(name, *storage))
        ROS_ERROR("Failed to get parameter \"%s\" from server\n", name.data());
}

template<typename T> 
void PathTracking::GetParam(const ros::NodeHandle &handle, const std::string &name,
                            const T &defaultValue, T* storage) const
{
    if (!handle.param<T>(name, *storage, defaultValue))
        ROS_WARN_STREAM("Failed to get parameter " << name.data() << " from server, setting default: " << defaultValue);
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