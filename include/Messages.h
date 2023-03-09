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

struct Params
{
    /* vehicle parameters */
    float carLength;
    float rearWheelsOffset;
    float frontWheelsOffset;

    /* speed controller parameters */
    float refSpeed;
    float speedP;
    float speedI;
    float speedMin;
    float speedMax;
    float speedRaiseRate;

    /* steering control parameters*/
    float steeringK;
    float steeringMin;
    float steeringMax;
    float lookAheadDistMin;
    float lookAheadDistMax;

    bool trackLoop;
};