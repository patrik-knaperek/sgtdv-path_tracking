/*****************************************************/
//Organization: Stuba Green Team
//Authors: Tereza Ábelová, Juraj Krasňanský
/*****************************************************/


#include "../include/TrackingAlgorithms.h"

TrackingAlgorithm::TrackingAlgorithm()
{
    m_coneIndexOffset = 0;
}

TrackingAlgorithm::~TrackingAlgorithm()
{

}

void TrackingAlgorithm::FreshTrajectory()
{
    m_coneIndexOffset = 0;
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

    if (m_coneIndexOffset >= msg.trajectory->points.size())
    {
        control.speed = 0.f;
        control.steeringAngle = 0.f;

        return control;
    }

    ComputeFrontWheelPos(msg.carPose);
    ComputeThetaDelta(msg.carPose->yaw, FindClosestPoint(msg.trajectory));
    
    if (m_coneIndexOffset >= msg.trajectory->points.size())
    {
        control.speed = 0.f;
        control.steeringAngle = 0.f;

        return control;
    }

    control.steeringAngle = ControlCommand(msg.speed);
    control.speed = msg.speed;

    return control;
}

void Stanley::ComputeFrontWheelPos(const sgtdv_msgs::CarPose::ConstPtr &carPose)
{
    const cv::Vec2f pos(carPose->position.x, carPose->position.y);
    m_frontWheelsPos = pos + cv::Vec2f(sinf(deg2rad(carPose->yaw)), cosf(deg2rad(carPose->yaw))) * FRONT_WHEELS_OFFSET;   
}

void Stanley::ComputeThetaDelta(float theta, const cv::Vec2f &closestPoint)
{
    const cv::Vec2f tangent(closestPoint - m_frontWheelsPos);
    m_thetaDelta = theta - rad2deg(atan2(tangent[1], tangent[0]));
}

cv::Vec2f Stanley::FindClosestPoint(const sgtdv_msgs::Point2DArr::ConstPtr &trajectory)
{
    for (size_t i = m_coneIndexOffset; i < trajectory->points.size(); i++)
    {
        cv::Vec2f point(trajectory->points[i].x, trajectory->points[i].y);

        if (static_cast<float>(cv::norm(m_frontWheelsPos - point)) < CLOSEST_POINT_THRESHOLD)
        {
            m_coneIndexOffset++;

            continue;
        }

        return point;
    }

    return cv::Vec2f(0.f, 0.f);
}

float Stanley::SpeedGain(float speed) const
{
    return CONTROL_GAIN / speed; //needs testing
}

float Stanley::ControlCommand(float speed) const
{
    return m_thetaDelta * SpeedGain(speed);
}