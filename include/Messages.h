/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/


#pragma once

#include <sgtdv_msgs/PathTrackingMsg.h>

struct PathTrackingMsg
{
    sgtdv_msgs::CarState::ConstPtr carState;
    sgtdv_msgs::Point2DArr::ConstPtr trajectory;
    float speed;
    float yawRate;
};

struct Control
{
    float speed;
    float steeringAngle;
};