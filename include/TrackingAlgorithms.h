/*****************************************************/
//Organization: Stuba Green Team
//Authors: Tereza Ábelová, Juraj Krasňanský, Patrik Knaperek
/*****************************************************/

#pragma once

/* C++ */
#include <cmath>
#include "opencv2/core/core.hpp"

/* ROS */
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

/* SGT */
#include <sgtdv_msgs/Point2D.h>
#include <sgtdv_msgs/Point2DArr.h>
#include <sgtdv_msgs/CarPose.h>
#include <sgtdv_msgs/CarVel.h>
#include <sgtdv_msgs/Control.h>
#include "../../SGT_Macros.h"
#include "../include/Messages.h"

constexpr float FPS = 60.f;
constexpr float TIME_PER_FRAME = 1.f / FPS;

#define deg2rad(x) (x*M_PI/180.f)
#define rad2deg(x) (x*180.f/M_PI)

class TrackingAlgorithm
{
public:
    struct Params
    {
        /* vehicle parameters */
        float carLength;
        float rearWheelsOffset;
        float frontWheelsOffset;

        /* speed PID controller parameters */
        // float refSpeed;
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

    virtual void Do(const PathTrackingMsg &msg, sgtdv_msgs::ControlPtr &controlMsg) = 0;
    virtual void SetParams(const Params &params)
    {
        m_params = params;
    };
    
#ifdef SGT_VISUALIZATION
    virtual void SetVisualizationPublishers(const ros::Publisher &targetPub, const ros::Publisher &steeringPosePub)
    {
        m_targetPub = targetPub;
        m_steeringPosePub = steeringPosePub;
    };
#endif /* SGT_VISUALIZATION */

    virtual void SetRefSpeed(const float refSpeed)
    {
        m_refSpeed = refSpeed;
    };
    void ResetIntegral()
    {
        m_speedIntegralError = 0.;
    };

protected:
    TrackingAlgorithm(const ros::NodeHandle &handle);
    ~TrackingAlgorithm() = default;

    virtual int8_t ComputeSpeedCommand(const float actSpeed, const int8_t speedCmdPrev);

#ifdef SGT_VISUALIZATION
    virtual void VisualizePoint(const cv::Vec2f point, const int point_id, const std::string& ns, const cv::Vec3f color) const;
    virtual void VisualizeSteering() const;
#endif /* SGT_VISUALIZATION */

    ros::Publisher m_targetPub;
    ros::Publisher m_steeringPosePub;
    Control m_control;
    Params m_params;

private:
    float m_refSpeed = 0.;
    float m_speedIntegralError = 0.;

};

class PurePursuit : public TrackingAlgorithm
{
public:
    PurePursuit(const ros::NodeHandle &handle);
    ~PurePursuit() = default;

    void Do(const PathTrackingMsg &msg, sgtdv_msgs::ControlPtr &controlMsg) override;

private:
    cv::Vec2f m_rearWheelsPos;
    float m_lookAheadDist;

    void ComputeRearWheelPos(const sgtdv_msgs::CarPose::ConstPtr &carPose);
    void ComputeLookAheadDist(const sgtdv_msgs::CarVel::ConstPtr &carVel);
    cv::Vec2f FindTargetPoint(const sgtdv_msgs::Point2DArr::ConstPtr &trajectory) const;
    float ComputeSteeringCommand(const PathTrackingMsg &msg, const cv::Vec2f &targetPoint);
};
