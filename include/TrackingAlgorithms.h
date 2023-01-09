/*****************************************************/
//Organization: Stuba Green Team
//Authors: Tereza Ábelová, Juraj Krasňanský
/*****************************************************/



#include "../include/Messages.h"
#include <ros/ros.h>
#include <cmath>
#include "opencv2/core/core.hpp"
#include <sgtdv_msgs/Point2D.h>
#include <visualization_msgs/Marker.h>

#define deg2rad(x) (x*M_PI/180.f)
#define rad2deg(x) (x*180.f/M_PI)

/*constexpr float CAR_LENGTH = 2.f;
constexpr float REAR_WHEELS_OFFSET = 0.9f;
constexpr float FRONT_WHEELS_OFFSET = 1.1f;
constexpr float CLOSEST_POINT_THRESHOLD = 0.2f;
constexpr float CONTROL_GAIN = 1.f;*/


class TrackingAlgorithm
{
protected:
    TrackingAlgorithm(ros::NodeHandle &handle);
    ~TrackingAlgorithm();

    virtual void LoadParams();
    virtual void VisualizeTargetPoint(float point_x, float point_y);

    size_t m_coneIndexOffset;   //where to start looking for nearest point
    ros::Publisher m_targetPub;
    ros::NodeHandle m_handle;
    Control m_control;

    // parameters
    float m_carLength;
    float m_rearWheelsOffset;
    float m_frontWheelsOffset;
    float m_closestPointTreshold;
    float m_controlGain;

    // controller parameters
    float m_speedP;
    float m_speedI;
    float m_speedD;
    float m_steeringP;
    float m_steeringI;
    float m_steeringD;

public:
    virtual Control Do(const PathTrackingMsg &msg) = 0;
    virtual void FreshTrajectory();
    virtual void SetPublisher(ros::Publisher targetPub);
    virtual void SetParams(float carLength, float rearWheelsOffset, float frontWheelsOffset, float closestPointTreshold, float controlGain);
    virtual void SetControllerParams(float speedP, float speedI, float speedD, float steerP, float steerI, float steerD);
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

};