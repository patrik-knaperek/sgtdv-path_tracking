/*****************************************************/
//Organization: Stuba Green Team
//Authors: Tereza Ábelová, Juraj Krasňanský
/*****************************************************/


#include "../include/Messages.h"
#include <cmath>
#include "opencv2/core/core.hpp"
#include <sgtdv_msgs/Point2D.h>

#define deg2rad(x) (x*M_PI/180.f)
#define rad2deg(x) (x*180.f/M_PI)

constexpr float CAR_LENGTH = 2.f;
constexpr float REAR_WHEELS_OFFSET = 0.9f;
constexpr float FRONT_WHEELS_OFFSET = 1.1f;
constexpr float CLOSEST_POINT_THRESHOLD = 0.2f;
constexpr float CONTROL_GAIN = 1.f;


class TrackingAlgorithm
{
protected:
    TrackingAlgorithm();
    ~TrackingAlgorithm();

    size_t m_coneIndexOffset;   //where to start looking for nearest point

public:
    virtual Control Do(const PathTrackingMsg &msg) = 0;
    virtual void FreshTrajectory();
private:
};



class Stanley : public TrackingAlgorithm
{
public:
    Stanley();
    ~Stanley();

    virtual Control Do(const PathTrackingMsg &msg);

private:
    cv::Vec2f m_frontWheelsPos;
    float m_thetaDelta;

    void ComputeFrontWheelPos(const sgtdv_msgs::CarState::ConstPtr &carState);
    void ComputeThetaDelta(float theta, const cv::Vec2f &closestPoint);
    cv::Vec2f FindClosestPoint(const sgtdv_msgs::Point2DArr::ConstPtr &trajectory);
    float ControlCommand(float speed) const;
    float SpeedGain(float speed) const;
};



class PurePursuit : public TrackingAlgorithm
{
public:
    PurePursuit();
    ~PurePursuit();

private:

};