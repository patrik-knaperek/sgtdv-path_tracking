/*****************************************************/
//Organization: Stuba Green Team
//Authors: Tereza Ábelová, Juraj Krasňanský, Patrik Knaperek
/*****************************************************/



#include "../include/Messages.h"
#include <ros/ros.h>
#include <cmath>
#include "opencv2/core/core.hpp"
#include <sgtdv_msgs/Point2D.h>
#include <visualization_msgs/Marker.h>

constexpr float FPS = 60.f;
constexpr float TIME_PER_FRAME = 1.f / FPS;

#define deg2rad(x) (x*M_PI/180.f)
#define rad2deg(x) (x*180.f/M_PI)

class TrackingAlgorithm
{
protected:
    TrackingAlgorithm(ros::NodeHandle &handle);
    ~TrackingAlgorithm();

    virtual void LoadParams();
    virtual void VisualizePoint(float point_x, float point_y, int point_id, cv::Vec3f color);
    virtual void ComputeSpeedCommand(const float actSpeed);

    size_t m_coneIndexOffset;   //where to start looking for nearest point
    ros::Publisher m_targetPub;
    ros::NodeHandle m_handle;
    Control m_control;
    
    float m_ramp = 0;
    double m_integralSpeed = 0;
    double m_previousSpeedError = 0;

    // parameters
    float m_carLength;
    float m_rearWheelsOffset;
    float m_frontWheelsOffset;
    float m_closestPointTreshold;
    float m_controlGain;
    float m_refSpeed;

    // controller parameters
    float m_speedP;
    float m_speedI;
    float m_speedD;
    int8_t m_speedMax;
    int8_t m_speedMin;

    float m_steeringP;
    float m_steeringI;
    float m_steeringD;
    float m_steeringMax;
    float m_steeringMin;

public:
    virtual Control Do(const PathTrackingMsg &msg) = 0;
    virtual void FreshTrajectory();
    virtual void SetPublisher(ros::Publisher targetPub);
    virtual void SetParams(float carLength, float rearWheelsOffset, float frontWheelsOffset, float closestPointTreshold, float controlGain, float constSpeed);
    virtual void SetControllerParams(float speedP, float speedI, float speedD, int8_t speedMax, int8_t speedMin,
                                     float steerP, float steerI, float steerD, float steerMax, float steerMin);
private:
};



class Stanley : public TrackingAlgorithm
{
public:
    Stanley(ros::NodeHandle &handle);
    ~Stanley();

    virtual Control Do(const PathTrackingMsg &msg);

private:
    cv::Vec2f m_frontWheelsPos;
    float m_thetaDelta;

    void ComputeFrontWheelPos(const sgtdv_msgs::CarPose::ConstPtr &carPose);
    void ComputeThetaDelta(float theta, const cv::Vec2f &closestPoint);
    cv::Vec2f FindClosestPoint(const sgtdv_msgs::Point2DArr::ConstPtr &trajectory);
    float ControlCommand(float speed) const;
    float SpeedGain(float speed) const;
};



class PurePursuit : public TrackingAlgorithm
{
public:
    PurePursuit(ros::NodeHandle &handle);
    ~PurePursuit();

    virtual Control Do(const PathTrackingMsg &msg);

private:
    sgtdv_msgs::Point2D FindTargetPoint(const PathTrackingMsg &msg);
    void ComputeSteeringCommand(const PathTrackingMsg &msg, const sgtdv_msgs::Point2D &targetPoint);

};