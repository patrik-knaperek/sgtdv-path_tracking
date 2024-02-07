/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/

#pragma once

/* C++ */
#include <chrono>

/* ROS */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

/* SGT */
#include <sgtdv_msgs/PathTrackingMsg.h>
#include <sgtdv_msgs/Control.h>
#include "../include/Messages.h"
#include "../include/TrackingAlgorithms.h"
#include <sgtdv_msgs/DebugState.h>
#include "../../SGT_Utils.h"

class PathTracking
{
public:
    PathTracking(const ros::NodeHandle &handle);
    ~PathTracking() = default;

    void LoadParams(const ros::NodeHandle &handle) const;
    void SetCmdPublisher(const ros::Publisher &cmdPub)
    {
        m_cmdPublisher = cmdPub;
    };
#ifdef SGT_VISUALIZATION
    void SetVisualizationPublishers(const ros::Publisher &targetPub, const ros::Publisher &steeringPosePub)
    {
        m_algorithm->SetVisualizationPublishers(targetPub, steeringPosePub);
    };
#endif /* SGT_VISUALIZATION */
#ifdef SGT_DEBUG_STATE
    void SetVisDebugPublisher(const ros::Publisher& publisher) { m_visDebugPublisher = publisher; };
#endif

    void Do(const PathTrackingMsg &msg);
    void StopVehicle();
    void StartVehicle();
    void SetRefSpeed(const float refSpeed)
    {
        m_algorithm->SetRefSpeed(refSpeed);
    }
private:
    ros::NodeHandle m_handle;
    ros::Publisher m_cmdPublisher;
    TrackingAlgorithm *m_algorithm;
    bool m_stopped;

#ifdef SGT_DEBUG_STATE
    ros::Publisher m_visDebugPublisher;
#endif
};