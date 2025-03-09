/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

#include "path_tracking.h"

void PathTracking::init(const Params& params)
{
  steering_control_obj_ = std::make_unique<SteeringControl>(SteeringControl(params.steering));
  speed_control_obj_ = std::make_unique<SpeedControl>(SpeedControl(params.speed));
}

void PathTracking::update(const sgtdv_msgs::PathTrackingMsg &msg, sgtdv_msgs::Control *cmd)
{ 
  if(stopped_)
  {
    cmd->speed = 0.0;
    cmd->steering_angle = 0.0;
  }
  else
  {
    cmd->steering_angle = steering_control_obj_->computeSteeringCommand(msg);
    
    const auto ref_speed = msg.trajectory.ref_speed.at(
      Utils::findClosestPointIdx(Eigen::Vector2d(msg.car_pose.position.x, msg.car_pose.position.y), 
      msg.trajectory.path.points, [&](const sgtdv_msgs::Point2D p)
      {
        return Eigen::Vector2d(p.x, p.y);
      }
    ));

    cmd->speed = speed_control_obj_->computeSpeedCommand(msg.car_vel.speed, ref_speed);

  }
}

void PathTracking::stopVehicle(void)
{
  if(!stopped_)
  {
    stopped_ = true;
    ROS_INFO("STOPPING VEHICLE");
  }
}

void PathTracking::startVehicle(void)
{
  if(stopped_)
  {
    stopped_ = false;
    speed_control_obj_->reinit();

    ROS_INFO("STARTING VEHICLE");
  }
}
