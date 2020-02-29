/*****************************************************/
//Organization: Stuba Green Team
//Authors: Tereza Ábelová, Juraj Krasňanský
/*****************************************************/


#include "../include/TrackingAlgorithms.h"

TrackingAlgorithm::TrackingAlgorithm()
{

}

TrackingAlgorithm::~TrackingAlgorithm()
{

}


Stanley::Stanley()
{
}

Stanley::~Stanley()
{

}

Control Stanley::Do(const PathTrackingMsg &msg)
{
    Control control;

    ComputeFrontWheelPos(msg.carState);
    ComputeThetaDelta(msg.carState->yaw, FindClosestPoint(msg.trajectory));
    
    control.steeringAngle = ControlCommand(msg.speed);
    control.speed = msg.speed;

    return control;
}

void Stanley::ComputeFrontWheelPos(const sgtdv_msgs::CarState::ConstPtr &carState)
{
    const cv::Vec2f pos(carState->position.x, carState->position.y);
    m_frontWheelsPos = pos + cv::Vec2f(sinf(deg2rad(carState->yaw)), cosf(deg2rad(carState->yaw))) * FRONT_WHEELS_OFFSET;
}

void Stanley::ComputeThetaDelta(float theta, const cv::Vec2f &closestPoint)
{
    const cv::Vec2f tangent(closestPoint - m_frontWheelsPos);
    m_thetaDelta = theta - rad2deg(atan2(tangent[1], tangent[0]));
}

cv::Vec2f Stanley::FindClosestPoint(const sgtdv_msgs::Point2DArr::ConstPtr &trajectory) const
{
    for (size_t i = 0; i < trajectory->points.size(); i++)
    {
        cv::Vec2f point(trajectory->points[i].x, trajectory->points[i].y);

        if (static_cast<float>(cv::norm(m_frontWheelsPos - point)) < CLOSEST_POINT_THRESHOLD)
        {
            continue;
        }

        return point;
    }
}

float Stanley::SpeedGain(float speed) const
{
    return CONTROL_GAIN / speed; //needs testing
}

float Stanley::ControlCommand(float speed) const
{
    return m_thetaDelta * SpeedGain(speed);
}