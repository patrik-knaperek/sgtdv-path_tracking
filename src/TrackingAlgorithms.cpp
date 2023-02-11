/*****************************************************/
//Organization: Stuba Green Team
//Authors: Tereza Ábelová, Juraj Krasňanský, Patrik Knaperek
/*****************************************************/


#include "../include/TrackingAlgorithms.h"

TrackingAlgorithm::TrackingAlgorithm(ros::NodeHandle &handle) :
    m_handle(handle)
{
    m_coneIndexOffset = 0;
    m_control.speed = 0;
    m_control.steeringAngle = 0;
    LoadParams();
}

TrackingAlgorithm::~TrackingAlgorithm()
{

}

void TrackingAlgorithm::LoadParams()
{
    // load parameters
    float carLength, rearWheelsOffset, frontWheelsOffset, closestPointTreshold, controlGain, refSpeed;
    std::cout << "LOADING PARAMETERS" << std::endl;
    if(!m_handle.getParam("/car_length", carLength))
        ROS_ERROR("Failed to get parameter \"/car_length\" from server\n");
    if(!m_handle.getParam("/rear_wheels_offset", rearWheelsOffset))
        ROS_ERROR("Failed to get parameter \"/rear_wheels_offset\" from server\n");
    if(!m_handle.getParam("/front_wheels_offset", frontWheelsOffset))
        ROS_ERROR("Failed to get parameter \"/front_wheels_offset\" from server\n");
    if(!m_handle.getParam("/closest_point_treshold", closestPointTreshold))
        ROS_ERROR("Failed to get parameter \"/closest_point_treshold\" from server\n");
    if(!m_handle.getParam("/control_gain", controlGain))
        ROS_ERROR("Failed to get parameter \"/control_gain\" from server\n");
    if(!m_handle.getParam("/ref_speed", refSpeed))
        ROS_ERROR("Failed to get parameter \"/const_speed\" from server\n");
    this->SetParams(carLength, rearWheelsOffset, frontWheelsOffset, closestPointTreshold, controlGain, refSpeed);

    // load controller parameters
    float speedP, speedI, speedD, steerP, speedMax, speedMin, steerI, steerD, steerMax, steerMin;
    if(!m_handle.getParam("controller/speed/p", speedP))
        ROS_ERROR("Failed to get parameter \"/controller/speed/p\" from server\n");
    if(!m_handle.getParam("controller/speed/i", speedI))
        ROS_ERROR("Failed to get parameter \"/controller/speed/i\" from server\n");
    if(!m_handle.getParam("controller/speed/d", speedD))
        ROS_ERROR("Failed to get parameter \"/controller/speed/d\" from server\n");
    if(!m_handle.getParam("controller/speed/max", speedMax))
        ROS_ERROR("Failed to get parameter \"/controller/speed/max\" from server\n");
    if(!m_handle.getParam("controller/speed/min", speedMin))
        ROS_ERROR("Failed to get parameter \"/controller/speed/min\" from server\n");
    if(!m_handle.getParam("controller/steering/p", steerP))
        ROS_ERROR("Failed to get parameter \"/controller/steering/p\" from server\n");
    if(!m_handle.getParam("controller/steering/i", steerI))
        ROS_ERROR("Failed to get parameter \"/controller/steering/i\" from server\n");
    if(!m_handle.getParam("controller/steering/d", steerD))
        ROS_ERROR("Failed to get parameter \"/controller/steering/d\" from server\n");
    if(!m_handle.getParam("controller/steering/max", steerMax))
        ROS_ERROR("Failed to get parameter \"/controller/steering/max\" from server\n");
    if(!m_handle.getParam("controller/steering/min", steerMin))
        ROS_ERROR("Failed to get parameter \"/controller/steering/min\" from server\n");
    this->SetControllerParams(speedP, speedI, speedD, static_cast<int8_t>(speedMax), static_cast<int8_t>(speedMin),
                             steerP, steerI, steerD, steerMax, steerMin);
}

void TrackingAlgorithm::FreshTrajectory()
{
    m_coneIndexOffset = 0;
}

void TrackingAlgorithm::SetPublisher(ros::Publisher targetPub)
{
    m_targetPub = targetPub;
}

void TrackingAlgorithm::SetParams(float carLength, float rearWheelsOffset, float frontWheelsOffset, float closestPointTreshold, float controlGain, float refSpeed)
{
    m_carLength = carLength;
    m_rearWheelsOffset = rearWheelsOffset;
    m_frontWheelsOffset = frontWheelsOffset;
    m_closestPointTreshold = closestPointTreshold;
    m_controlGain = controlGain;
    m_refSpeed = refSpeed;
}

void TrackingAlgorithm::SetControllerParams(float speedP, float speedI, float speedD, int8_t speedMax, int8_t speedMin,
                                            float steerP, float steerI, float steerD, float steerMax, float steerMin)
{
    m_speedP = speedP;
    m_speedI = speedI;
    m_speedD = speedD;
    m_speedMax = speedMax;
    m_speedMin = speedMin;

    m_steeringP = steerP;
    m_steeringI = steerI;
    m_steeringD = steerD;
    m_steeringMax = steerMax;
    m_steeringMin = steerMin;
}

void TrackingAlgorithm::VisualizePoint(float p_x, float p_y, int p_id, cv::Vec3f color)
{
    visualization_msgs::Marker marker;
    
    marker.color.r              = color(0);
    marker.color.g              = color(1);
    marker.color.b              = color(2);
    marker.color.a              = 1.0;
    marker.pose.position.x      = p_x;
    marker.pose.position.y      = p_y;
    marker.pose.orientation.w   = 1.0;
    marker.type                 = visualization_msgs::Marker::SPHERE;
    marker.action               = visualization_msgs::Marker::ADD;
    marker.id                   = p_id;
    marker.scale.x              = 0.5;
    marker.scale.y              = 0.5;
    marker.scale.z              = 0.5;
    marker.header.stamp         = ros::Time::now();
    marker.header.frame_id      = "map";
    m_targetPub.publish(marker);
}

void TrackingAlgorithm::ComputeSpeedCommand(const float actSpeed)
{
    // regulation error
    const double speedError = m_refSpeed - actSpeed;
    if (m_ramp < 1)
    {
        m_ramp += 0.1;
    }

    // integral
    if (m_control.speed < m_speedMax)   // Anti-windup
    {
        m_integralSpeed += speedError * TIME_PER_FRAME;
    }
    
    // derivative
    const double derivativeSpeed = (speedError - m_previousSpeedError) / TIME_PER_FRAME;
    m_previousSpeedError = speedError;

    // P + I + D
    m_control.speed = static_cast<uint8_t>(m_ramp * (m_speedP * speedError + m_speedI * m_integralSpeed + m_speedD * derivativeSpeed));
    
    // saturation
    if (m_control.speed > m_speedMax)
    {
        m_control.speed = m_speedMax;
    } else if (m_control.speed < m_speedMin)
    {
        m_control.speed = m_speedMin;
    }
}

Stanley::Stanley(ros::NodeHandle &handle) :
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
    ComputeThetaDelta(msg.carPose->yaw, FindClosestPoint(msg.trajectory));
    
    if (m_coneIndexOffset >= msg.trajectory->points.size())
    {
        m_control.speed = 0.f;
        m_control.steeringAngle = 0.f;

        return m_control;
    }

    m_control.steeringAngle = ControlCommand(msg.carVel->speed);
    // saturation
    if (m_control.steeringAngle > m_steeringMax)
    {
        m_control.steeringAngle = m_steeringMax;
    } else if (m_control.steeringAngle < m_steeringMin)
    {
        m_control.steeringAngle = m_steeringMin;
    }

    this->ComputeSpeedCommand(msg.carVel->speed);

    return m_control;
}

void Stanley::ComputeFrontWheelPos(const sgtdv_msgs::CarPose::ConstPtr &carPose)
{
    const cv::Vec2f pos(carPose->position.x, carPose->position.y);
    m_frontWheelsPos = pos + cv::Vec2f(cosf(carPose->yaw), sinf(carPose->yaw)) * m_frontWheelsOffset;
    VisualizePoint(m_frontWheelsPos(0), m_frontWheelsPos(1), 2, cv::Vec3f(0.0, 0.0, 1.0));
}

void Stanley::ComputeThetaDelta(float theta, const cv::Vec2f &closestPoint)
{
    const cv::Vec2f tangent(closestPoint - m_frontWheelsPos);
    m_thetaDelta = theta - atan2(tangent[1], tangent[0]);
}

cv::Vec2f Stanley::FindClosestPoint(const sgtdv_msgs::Point2DArr::ConstPtr &trajectory)
{
    for (size_t i = m_coneIndexOffset; i < trajectory->points.size(); i++)
    {
        cv::Vec2f pointA(trajectory->points[i].x, trajectory->points[i].y);
        cv::Vec2f pointB(trajectory->points[i+1].x, trajectory->points[i+1].y);

        if (static_cast<float>(cv::norm(m_frontWheelsPos - pointA)) > static_cast<float>(cv::norm(m_frontWheelsPos - pointB)) ||
            static_cast<float>(cv::norm(m_frontWheelsPos - pointA)) < m_closestPointTreshold)
        {
            m_coneIndexOffset++;

            continue;
        }
        VisualizePoint(pointA[0], pointA[1], 0, cv::Vec3f(1.0, 0.0, 0.0));
        VisualizePoint(pointB[0], pointB[1], 1, cv::Vec3f(1.0, 1.0, 0.0));
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
}

PurePursuit::PurePursuit(ros::NodeHandle &handle) :
    TrackingAlgorithm(handle)
{

}

PurePursuit::~PurePursuit()
{

}

Control PurePursuit::Do(const PathTrackingMsg &msg)
{    
    sgtdv_msgs::Point2D targetPoint = FindTargetPoint(msg);
    this->VisualizePoint(targetPoint.x, targetPoint.y, 0, cv::Vec3f(1.0, 0.0, 0.0));
    this->ComputeSteeringCommand(msg, targetPoint);
    this->ComputeSpeedCommand(msg.carVel->speed);

    return m_control;
}

sgtdv_msgs::Point2D PurePursuit::FindTargetPoint(const PathTrackingMsg &msg)
{
    const auto centerLineIt  = std::min_element(msg.trajectory->points.begin(), msg.trajectory->points.end(),
                                                [&](const sgtdv_msgs::Point2D &a,
                                                const sgtdv_msgs::Point2D &b) {
                                                const double da = std::hypot(msg.carPose->position.x - a.x,
                                                                            msg.carPose->position.y - a.y);
                                                const double db = std::hypot(msg.carPose->position.x - b.x,
                                                                            msg.carPose->position.y - b.y);

                                                return da < db;
                                                });
    const auto centerLineIdx = std::distance(msg.trajectory->points.begin(), centerLineIt);
    const auto size          = msg.trajectory->points.size();
    const auto nextIdx       = (centerLineIdx + 10) % size;

    return msg.trajectory->points[nextIdx];
}

void PurePursuit::ComputeSteeringCommand(const PathTrackingMsg &msg, const sgtdv_msgs::Point2D &targetPoint)
{
    const double theta = msg.carPose->yaw;
    const double alpha = std::atan2((targetPoint.y - msg.carPose->position.y),(targetPoint.x - msg.carPose->position.x)) - theta;
    const double lookAheadDist = std::hypot((targetPoint.y - msg.carPose->position.y), (targetPoint.x - msg.carPose->position.x));
    const double steeringAngleAct = m_control.steeringAngle;
    const double steeringAngleWish = std::atan2(2*std::sin(alpha)*m_carLength,lookAheadDist);
    const double steeringAngleError = steeringAngleWish - steeringAngleAct;
    
    m_control.steeringAngle = static_cast<float>(m_steeringP * steeringAngleError);

    // saturation
    if (m_control.steeringAngle > m_steeringMax)
    {
        m_control.steeringAngle = m_steeringMax;
    } else if (m_control.steeringAngle < m_steeringMin)
    {
        m_control.steeringAngle = m_steeringMin;
    }
}