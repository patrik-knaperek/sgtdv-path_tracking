/*****************************************************/
//Organization: Stuba Green Team
//Authors: Tereza Ábelová, Juraj Krasňanský, Patrik Knaperek
/*****************************************************/

#include "../include/Messages.h"
#include <ros/ros.h>
#include <cmath>
#include "opencv2/core/core.hpp"
#include <sgtdv_msgs/Point2D.h>
#include <sgtdv_msgs/Point2DArr.h>
#include <sgtdv_msgs/CarPose.h>
#include <sgtdv_msgs/CarVel.h>
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

    virtual void VisualizePoint(const cv::Vec2f point, const int point_id, const cv::Vec3f color) const;
    virtual void ComputeSpeedCommand(const float actSpeed);

    //size_t m_coneIndexOffset;   //where to start looking for nearest point
    ros::Publisher m_targetPub;
    Control m_control;

    /* vehicle parameters */
    float m_carLength;
    float m_rearWheelsOffset;
    float m_frontWheelsOffset;

    /* controller parameters */
    float m_refSpeed;
    float m_speedP;
    float m_speedI;
    std::vector<int8_t> m_speedRange;
    float m_speedRaiseRate;
    float m_steeringK;
    std::vector<float> m_steeringRange;
    std::vector<float> m_lookAheadDistRange;

    bool m_trackLoop;
    
public:
    virtual Control Do(const PathTrackingMsg &msg) = 0;
    virtual void SetParams(const Params &params);
    //virtual void FreshTrajectory();
    virtual void SetPublisher(ros::Publisher targetPub);
private:
};

/*class Stanley : public TrackingAlgorithm
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
    cv::Vec2f FindTargetPoint(const sgtdv_msgs::Point2DArr::ConstPtr &trajectory);
    float ControlCommand(float speed) const;
    float SpeedGain(float speed) const;
};*/

class PurePursuit : public TrackingAlgorithm
{
public:
    PurePursuit(ros::NodeHandle &handle);
    ~PurePursuit();

    virtual Control Do(const PathTrackingMsg &msg);

private:
    cv::Vec2f m_rearWheelsPos;
    float m_lookAheadDist;

    void ComputeRearWheelPos(const sgtdv_msgs::CarPose::ConstPtr &carPose);
    float ComputeLookAheadDist(const sgtdv_msgs::CarVel::ConstPtr &carVel);
    cv::Vec2f FindTargetPoint(const sgtdv_msgs::Point2DArr::ConstPtr &trajectory);
    void ComputeSteeringCommand(const PathTrackingMsg &msg, const cv::Vec2f &targetPoint);
};