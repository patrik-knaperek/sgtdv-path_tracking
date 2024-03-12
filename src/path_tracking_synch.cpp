/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/


#include "../include/path_tracking_synch.h"

PathTrackingSynch::PathTrackingSynch(const ros::NodeHandle &handle) :
      path_tracking_obj_(handle)
    , trajectory_ready_(false)
    , pose_ready_(false)
    , velocity_ready_(false)
{
    
}

void PathTrackingSynch::doPlannedTrajectory(const sgtdv_msgs::Point2DArr::ConstPtr &msg)
{
    path_tracking_msg_.trajectory = msg;
    trajectory_ready_ = true;
}

void PathTrackingSynch::doPoseEstimate(const sgtdv_msgs::CarPose::ConstPtr &msg)
{
    path_tracking_msg_.car_pose = msg;
    pose_ready_ = true;
}

void PathTrackingSynch::doVelocityEstimate(const sgtdv_msgs::CarVel::ConstPtr &msg)
{
    path_tracking_msg_.car_vel = msg;
    velocity_ready_ = true;
}

bool PathTrackingSynch::stopCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    path_tracking_obj_.stopVehicle();
    return true;
}
bool PathTrackingSynch::startCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    path_tracking_obj_.startVehicle();
    return true;
}

bool PathTrackingSynch::setSpeedCallback(sgtdv_msgs::Float32Srv::Request &req, sgtdv_msgs::Float32Srv::Response &res)
{
    path_tracking_obj_.setRefSpeed(req.data);
    return 1;
}

void PathTrackingSynch::update()
{
    
    if(trajectory_ready_ && pose_ready_ && velocity_ready_)
    {
        path_tracking_obj_.update(path_tracking_msg_);
    }
}
