/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/


#pragma once

#include <sgtdv_msgs/PathTrackingMsg.h>

struct PathTrackingMsg
{
    sgtdv_msgs::CarPose::ConstPtr car_pose;
    sgtdv_msgs::Point2DArr::ConstPtr trajectory;
    sgtdv_msgs::CarVel::ConstPtr car_vel;
};

struct Control
{
    int8_t speed;
    float steering_angle;
};
