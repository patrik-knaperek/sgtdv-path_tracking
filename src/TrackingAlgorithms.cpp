/*****************************************************/
//Organization: Stuba Green Team
//Authors: Tereza Ábelová, Juraj Krasňanský, Patrik Knaperek
/*****************************************************/


#include "../include/TrackingAlgorithms.h"

TrackingAlgorithm::TrackingAlgorithm(const ros::NodeHandle &handle)
{
    m_control.speed = 0;
    m_control.steeringAngle = 0;
}

void TrackingAlgorithm::VisualizePoint(const cv::Vec2f point, const int p_id, const cv::Vec3f color) const
{
    visualization_msgs::Marker marker;
    
    marker.color.r              = color(0);
    marker.color.g              = color(1);
    marker.color.b              = color(2);
    marker.color.a              = 1.0;
    marker.pose.position.x      = point[0];
    marker.pose.position.y      = point[1];
    marker.pose.orientation.w   = 1.0;
    marker.type                 = visualization_msgs::Marker::SPHERE;
    marker.action               = visualization_msgs::Marker::ADD;
    marker.id                   = p_id;
    marker.ns                   = std::to_string(p_id);
    marker.scale.x              = 0.5;
    marker.scale.y              = 0.5;
    marker.scale.z              = 0.5;
    marker.header.stamp         = ros::Time::now();
    marker.header.frame_id      = "map";
    m_targetPub.publish(marker);
}

void TrackingAlgorithm::VisualizeSteering() const
{
    geometry_msgs::PoseStamped steeringPose;
    steeringPose.header.stamp = ros::Time::now();
    steeringPose.header.frame_id = std::string("base_link");
    steeringPose.pose.position.x  = m_params.frontWheelsOffset;

    steeringPose.pose.orientation.z = sin((m_control.steeringAngle) / 2);
    steeringPose.pose.orientation.w = cos((m_control.steeringAngle) / 2);

    m_steeringPosePub.publish(steeringPose);
}

void TrackingAlgorithm::ComputeSpeedCommand(const float actSpeed)
{
    static float speedCmdAct = 0.f;
    static float speedCmdPrev = 0.f;
    // regulation error
    const double speedError = m_params.refSpeed - actSpeed;

    // proportional
    speedCmdAct = m_params.speedP * speedError;

    // integral
    static double integralSpeed = 0.0;
    if (m_params.speedI)
    {
        if (m_control.speed < m_params.speedMax)   // Anti-windup
        {
        integralSpeed += speedError * TIME_PER_FRAME;
        }
        speedCmdAct += m_params.speedI * integralSpeed;
    }

    // ramp
    static ros::Time lastRaise = ros::Time::now();

        if (speedCmdAct - speedCmdPrev > 1.0)
        {
            if ((ros::Time::now() - lastRaise) - ros::Duration(1 / m_params.speedRaiseRate) > ros::Duration(0))
            {
                speedCmdPrev = ++m_control.speed;
                lastRaise = ros::Time::now();
            } else
            {
                m_control.speed = speedCmdPrev;
            }
        } else
        {
            m_control.speed = static_cast<int8_t>(speedCmdAct);
            speedCmdPrev = m_control.speed;
        }
    
    // saturation
    if (m_control.speed > m_params.speedMax)
    {
        m_control.speed = m_params.speedMax;
    } else if (m_control.speed < m_params.speedMin)
    {
        m_control.speed = m_params.speedMin;
    }
}

/*Stanley::Stanley(ros::NodeHandle &handle) :
    TrackingAlgorithm(handle)
{

}

Stanley::~Stanley()
{

}

Control Stanley::Do(const PathTrackingMsg &msg)
{
    if (m_coneIndexOffset >= msg.trajectory->points.size())
    {
        m_control.speed = 0.f;
        m_control.steeringAngle = 0.f;

        return m_control;
    }

    ComputeFrontWheelPos(msg.carPose);
    ComputeThetaDelta(msg.carPose->yaw, FindTargetPoint(msg.trajectory));
    
    if (m_coneIndexOffset >= msg.trajectory->points.size())
    {
        m_control.speed = 0.f;
        m_control.steeringAngle = 0.f;

        return m_control;
    }

    m_control.steeringAngle = ControlCommand(msg.carVel->speed);
    // saturation
    if (m_control.steeringAngle > m_speedRange[1])
    {
        m_control.steeringAngle = m_speedRange[1];
    } else if (m_control.steeringAngle < m_speedRange[0])
    {
        m_control.steeringAngle = m_speedRange[0];
    }

    this->ComputeSpeedCommand(msg.carVel->speed);

    return m_control;
}

void Stanley::ComputeFrontWheelPos(const sgtdv_msgs::CarPose::ConstPtr &carPose)
{
    const cv::Vec2f pos(carPose->position.x, carPose->position.y);
    m_frontWheelsPos = pos + cv::Vec2f(cosf(carPose->yaw), sinf(carPose->yaw)) * m_frontWheelsOffset;
    VisualizePoint(m_frontWheelsPos, 2, cv::Vec3f(0.0, 0.0, 1.0));
}

void Stanley::ComputeThetaDelta(float theta, const cv::Vec2f &closestPoint)
{
    const cv::Vec2f tangent(closestPoint - m_frontWheelsPos);
    m_thetaDelta = theta - atan2(tangent[1], tangent[0]);
}

cv::Vec2f Stanley::FindTargetPoint(const sgtdv_msgs::Point2DArr::ConstPtr &trajectory)
{
    for (size_t i = m_coneIndexOffset; i < trajectory->points.size(); i++)
    {
        cv::Vec2f pointA(trajectory->points[i].x, trajectory->points[i].y);
        cv::Vec2f pointB(trajectory->points[i+1].x, trajectory->points[i+1].y);

        if (static_cast<float>(cv::norm(m_frontWheelsPos - pointA)) > static_cast<float>(cv::norm(m_frontWheelsPos - pointB)) ||
            static_cast<float>(cv::norm(m_frontWheelsPos - pointA)) < m_lookAheadDist)
        {
            m_coneIndexOffset++;

            continue;
        }
        VisualizePoint(pointA, 0, cv::Vec3f(1.0, 0.0, 0.0));
        VisualizePoint(pointB, 1, cv::Vec3f(1.0, 1.0, 0.0));
        return pointA;
    }

    return cv::Vec2f(0.f, 0.f);
}

float Stanley::SpeedGain(float speed) const
{
    if (speed != 0)
    {
        return m_controlGain / speed; //needs testing
    } 
    else
        return 1;
    
}

float Stanley::ControlCommand(float speed) const
{
    return m_thetaDelta * SpeedGain(speed);
}*/

PurePursuit::PurePursuit(const ros::NodeHandle &handle) :
    TrackingAlgorithm(handle)
{

}

Control PurePursuit::Do(const PathTrackingMsg &msg)
{    
    ComputeRearWheelPos(msg.carPose);
    ComputeLookAheadDist(msg.carVel);
    const cv::Vec2f targetPoint = FindTargetPoint(msg.trajectory);
    //VisualizePoint(targetPoint, 0, cv::Vec3f(1.0, 0.0, 0.0));
    ComputeSteeringCommand(msg, targetPoint);
    ComputeSpeedCommand(msg.carVel->speed);
    ComputeSpeedCommand(msg.carVel->speed);
    ComputeSpeedCommand(msg.carVel->speed);

    return m_control;
}

void PurePursuit::ComputeRearWheelPos(const sgtdv_msgs::CarPose::ConstPtr &carPose)
{
    const cv::Vec2f pos(carPose->position.x, carPose->position.y);
    m_rearWheelsPos = pos - cv::Vec2f(cosf(carPose->yaw), sinf(carPose->yaw)) * m_params.rearWheelsOffset;
    VisualizePoint(m_rearWheelsPos, 1, cv::Vec3f(0.0, 0.0, 1.0));
}

void PurePursuit::ComputeLookAheadDist(const sgtdv_msgs::CarVel::ConstPtr &carVel)
{
    const float lookAheadDist = m_params.steeringK * carVel->speed;
    if (lookAheadDist < m_params.lookAheadDistMin)
    {
        m_lookAheadDist = m_params.lookAheadDistMin;
    } else if (lookAheadDist > m_params.lookAheadDistMax)
    {
        m_lookAheadDist = m_params.lookAheadDistMax;
    } else
    {
        m_lookAheadDist = lookAheadDist;
    }
}

cv::Vec2f PurePursuit::FindTargetPoint(const sgtdv_msgs::Point2DArr::ConstPtr &trajectory) const
{
    const auto centerLineIt  = std::min_element(trajectory->points.begin(), trajectory->points.end(),
                                                [&](const sgtdv_msgs::Point2D &a,
                                                const sgtdv_msgs::Point2D &b) {
                                                const double da = std::hypot(m_rearWheelsPos[0] - a.x,
                                                                            m_rearWheelsPos[1] - a.y);
                                                const double db = std::hypot(m_rearWheelsPos[0] - b.x,
                                                                            m_rearWheelsPos[1] - b.y);

                                                return da < db;
                                                });
    const auto centerLineIdx = std::distance(trajectory->points.begin(), centerLineIt);
    const auto size          = trajectory->points.size();

    static int offset;
    static int nextIdx;
    static cv::Vec2f targetPoint;

    offset = 0;
    do
    {
        if (!m_params.trackLoop)
        {
            nextIdx = (centerLineIdx + offset++);
            if (nextIdx > size - 1) break;
        } else {
            nextIdx = (centerLineIdx + offset++) % size;
        }
        targetPoint[0] = trajectory->points[nextIdx].x;
        targetPoint[1] = trajectory->points[nextIdx].y;
    } while (cv::norm(m_rearWheelsPos - targetPoint) < m_lookAheadDist);

    VisualizePoint(targetPoint, 0, cv::Vec3f(1.0, 0.0, 0.0));
    VisualizePoint(cv::Vec2f(trajectory->points[centerLineIdx].x, trajectory->points[centerLineIdx].y), 2, cv::Vec3f(1.0, 1.0, 0.0));
    return targetPoint;
}

void PurePursuit::ComputeSteeringCommand(const PathTrackingMsg &msg, const cv::Vec2f &targetPoint)
{
    const double theta = msg.carPose->yaw;
    const double alpha = std::atan2((targetPoint[1] - msg.carPose->position.y),(targetPoint[0] - msg.carPose->position.x)) - theta;
    m_control.steeringAngle = static_cast<float>(std::atan2(2*std::sin(alpha)*m_params.carLength,m_lookAheadDist));

    // saturation
    if (m_control.steeringAngle > m_params.steeringMax)
    {
        m_control.steeringAngle = m_params.steeringMax;
    } else if (m_control.steeringAngle < m_params.steeringMin)
    {
        m_control.steeringAngle = m_params.steeringMin;
    }

    VisualizeSteering();
}