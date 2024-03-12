/*****************************************************/
//Organization: Stuba Green Team
//Authors: Tereza Ábelová, Juraj Krasňanský
/*****************************************************/


#include "../include/path_tracking.h"

PathTracking::PathTracking(const ros::NodeHandle &handle) :
  handle_(handle)
, algorithm_(new PurePursuit(handle))
, stopped_(true)
{
    //algorithm_ = new Stanley(handle); // malfunctioning, needs fix
    loadParams(handle);
}

void PathTracking::loadParams(const ros::NodeHandle &handle) const
{
    ROS_INFO("LOADING PARAMETERS");
    TrackingAlgorithm::Params params;
    
    /* load vehicle parameters */
    Utils::loadParam(handle, "/vehicle/car_length", &params.car_length);
    Utils::loadParam(handle, "/vehicle/rear_wheels_offset", &params.rear_wheels_offset);
    Utils::loadParam(handle, "/vehicle/front_wheels_offset", &params.front_wheels_offset);
    
    /* load PID controller parameters */
    Utils::loadParam(handle, "/controller/speed/p", &params.speed_p);
    Utils::loadParam(handle, "controller/speed/i", &params.speed_i);
    Utils::loadParam(handle, "/controller/speed/min", &params.speed_min);
    Utils::loadParam(handle, "/controller/speed/max", &params.speed_max);
    // Utils::loadParam(handle, "/controller/speed/ref_speed", &params.refSpeed);
    Utils::loadParam(handle, "/controller/speed/speed_raise_rate", &params.speed_raise_rate);
    Utils::loadParam(handle, "/controller/steering/k", &params.steering_k);
    Utils::loadParam(handle, "/controller/steering/min", &params.steering_min);
    Utils::loadParam(handle, "/controller/steering/max", &params.steering_max);
    Utils::loadParam(handle, "/controller/steering/lookahead_dist_min", &params.lookahead_dist_min);
    Utils::loadParam(handle, "/controller/steering/lookahead_dist_max", &params.lookahead_dist_max);

    Utils::loadParam(handle, "/track_loop", true, &params.track_loop);
    algorithm_->setParams(params);
    
}

void PathTracking::stopVehicle()
{
    if (!stopped_)
    {
	    stopped_ = true;
    	ROS_INFO("STOPPING VEHICLE");
    }
}

void PathTracking::startVehicle()
{
    loadParams(handle_);
    algorithm_->resetIntegral();
    stopped_ = false;
    ROS_INFO("STARTING VEHICLE");
}

void PathTracking::update(const PathTrackingMsg &msg)
{
#ifdef SGT_DEBUG_STATE
	sgtdv_msgs::DebugState state;
	state.stamp = ros::Time::now();
	state.working_state = 1;
	vis_debug_pub_.publish(state);
#endif // SGT_DEBUG_STATE

    static boost::shared_ptr<sgtdv_msgs::Control> control_msg = boost::make_shared<sgtdv_msgs::Control>();
    if (stopped_)
    {
        control_msg->speed = 0.0;
        control_msg->steering_angle = 0.0;
    } else
    {
        algorithm_->update(msg, control_msg);
    }

    control_msg->stamp = ros::Time::now();
    cmd_pub_.publish(control_msg);

#ifdef SGT_DEBUG_STATE
	state.stamp = ros::Time::now();
	state.working_state = 0;
    state.speed = control_msg->speed;
    state.angle = control_msg->steering_angle;
	vis_debug_pub_.publish(state);
#endif // SGT_DEBUG_STATE
}