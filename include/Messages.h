/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/


#pragma once

#include <sgtdv_msgs/PathTrackingMsg.h>

struct PathTrackingMsg
{
    sgtdv_msgs::CarPose::ConstPtr carPose;
    sgtdv_msgs::Point2DArr::ConstPtr trajectory;
    sgtdv_msgs::CarVel::ConstPtr carVel;
};

struct Control
{
    int8_t speed;
    float steeringAngle;
};
