/*****************************************************/
//Organization: Stuba Green Team
//Authors: Tereza Ábelová, Juraj Krasňanský, Patrik Knaperek
/*****************************************************/


#include "../include/TrackingAlgorithms.h"

TrackingAlgorithm::TrackingAlgorithm(const ros::NodeHandle &handle)
{
    
}

#ifdef SGT_VISUALIZATION
void TrackingAlgorithm::VisualizePoint(const cv::Vec2f point, const int p_id, const std::string& ns, const cv::Vec3f color) const
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
    marker.ns                   = ns;
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
#endif /* SGT_VISUALIZATION */

int8_t TrackingAlgorithm::ComputeSpeedCommand(const float actSpeed, const int8_t speedCmdPrev)
{
    ROS_DEBUG_STREAM("ref speed: " << m_refSpeed);
	ROS_DEBUG_STREAM("speed: " << actSpeed);
	static float speedCmdAct = 0.f;
    static double lastRaise = ros::Time::now().toSec();

    // regulation error
    const double speedError = m_refSpeed - actSpeed;

    // proportional
    speedCmdAct = m_params.speedP * speedError;

    // integral
    if (m_params.speedI)
    {
        if (speedCmdPrev < m_params.speedMax)   // Anti-windup
        {
            m_speedIntegralError += speedError * TIME_PER_FRAME;
        }
        speedCmdAct += m_params.speedI * m_speedIntegralError;
    }

    // ramp
    // const double speedCmdDelta = speedCmdAct - speedCmdPrev;
    // if (std::abs(speedCmdDelta) > 1.0)
    // {
    //     if ((ros::Time::now().toSec() - lastRaise) > 1. / m_params.speedRaiseRate)
    //     {
    //         speedCmdAct = speedCmdPrev + (speedCmdDelta) / std::abs(speedCmdDelta);
    //         lastRaise = ros::Time::now().toSec();
    //     } else
    //     {
    //         speedCmdAct = speedCmdPrev;

    //     }
    // }
    
    // saturation
    speedCmdAct = std::max(m_params.speedMin, std::min(speedCmdAct, m_params.speedMax));
    
    return static_cast<int8_t>(speedCmdAct);
}

PurePursuit::PurePursuit(const ros::NodeHandle &handle) :
    TrackingAlgorithm(handle)
{

}

void PurePursuit::Do(const PathTrackingMsg &msg, sgtdv_msgs::ControlPtr &controlMsg)
{    
    ComputeRearWheelPos(msg.carPose);
    ComputeLookAheadDist(msg.carVel);
    const cv::Vec2f targetPoint = FindTargetPoint(msg.trajectory);
    controlMsg->steeringAngle = ComputeSteeringCommand(msg, targetPoint); 
    controlMsg->speed = ComputeSpeedCommand(msg.carVel->speed, controlMsg->speed);
}

void PurePursuit::ComputeRearWheelPos(const sgtdv_msgs::CarPose::ConstPtr &carPose)
{
    const cv::Vec2f pos(carPose->position.x, carPose->position.y);
    m_rearWheelsPos = pos - cv::Vec2f(cosf(carPose->yaw) * m_params.rearWheelsOffset, sinf(carPose->yaw)  * m_params.rearWheelsOffset);
#ifdef SGT_VISUALIZATION
    VisualizePoint(m_rearWheelsPos, 1, "rear wheels" , cv::Vec3f(0.0, 0.0, 1.0));
#endif /* SGT_VISUALIZATION */
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
    static int nextIdx = 0, prevIdx;
    static cv::Vec2f targetPoint, nextPoint;

    offset = 0;
    targetPoint(0) = trajectory->points[centerLineIdx].x;
    targetPoint(1) = trajectory->points[centerLineIdx].y;
    while (1)
    {
        if (!m_params.trackLoop)
        {
            nextIdx = (centerLineIdx + offset++);
            if (nextIdx > size - 1) break;
        } else {
            nextIdx = (centerLineIdx + offset++) % size;
        }
        nextPoint(0) = trajectory->points[nextIdx].x;
        nextPoint(1) = trajectory->points[nextIdx].y;

        if (cv::norm(m_rearWheelsPos - nextPoint) < m_lookAheadDist)
        {
            targetPoint = nextPoint;
            continue;
        }
        else
        {
            break;
        }
    }

    /* compute goal position exactly in look-ahead distance from the vehicle's position,
    *  interpolated on line given by the trajectory points 
    */
    if (targetPoint != nextPoint)
    {
        const auto slopeAngle = std::atan2(
            nextPoint(1) - targetPoint(1),
            nextPoint(0) - targetPoint(0)
        );
        
        const auto bearingVector = targetPoint - m_rearWheelsPos;
        const auto d = cv::norm(bearingVector);
        const auto theta = std::atan2(bearingVector(1), bearingVector(0));
        const auto gamma = M_PI - slopeAngle + theta;

        const auto x = 
            d * cos(gamma) + sqrt(std::pow(d,2) * std::pow(cos(gamma),2) - std::pow(d,2) + std::pow(m_lookAheadDist,2));

        targetPoint(0) += cos(slopeAngle) * x;
        targetPoint(1) += sin(slopeAngle) * x;
    }
#ifdef SGT_VISUALIZATION
    VisualizePoint(targetPoint, 0, "target point", cv::Vec3f(1.0, 0.0, 0.0));
    VisualizePoint(cv::Vec2f(trajectory->points[centerLineIdx].x, trajectory->points[centerLineIdx].y), 2, "closest point", cv::Vec3f(1.0, 1.0, 0.0));
#endif /* SGT_VISUALIZATION */

    return targetPoint;
}

float PurePursuit::ComputeSteeringCommand(const PathTrackingMsg &msg, const cv::Vec2f &targetPoint)
{
    static float steeringAngle = 0.0;
    const double theta = msg.carPose->yaw;
    const double alpha = std::atan2((targetPoint[1] - msg.carPose->position.y),(targetPoint[0] - msg.carPose->position.x)) - theta;
    steeringAngle = static_cast<float>(std::atan2(2*std::sin(alpha)*m_params.carLength,m_lookAheadDist));

    // saturation
    if (steeringAngle > m_params.steeringMax)
    {
        steeringAngle = m_params.steeringMax;
    } else if (steeringAngle < m_params.steeringMin)
    {
        steeringAngle = m_params.steeringMin;
    }

#ifdef SGT_VISUALIZATION
    VisualizeSteering();
#endif /* SGT_VISUALIZATION */

    return steeringAngle;
}
