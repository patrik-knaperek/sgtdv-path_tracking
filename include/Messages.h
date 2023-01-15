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
    float speed;
    float yawRate;
};

struct Control
{
    int8_t speed;
    float steeringAngle;
};