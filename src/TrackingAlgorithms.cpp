/*****************************************************/
//Organization: Stuba Green Team
//Authors: Tereza Ábelová, Juraj Krasňanský
/*****************************************************/


#include "../include/TrackingAlgorithms.h"

TrackingAlgorithm::TrackingAlgorithm(ros::NodeHandle &handle) :
    m_handle(handle)
{
    m_coneIndexOffset = 0;
    LoadParams();
}

TrackingAlgorithm::~TrackingAlgorithm()
{

}

void TrackingAlgorithm::LoadParams()
{
    // load parameters
    float carLength, rearWheelsOffset, frontWheelsOffset, closestPointTreshold, controlGain;
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
    this->SetParams(carLength, rearWheelsOffset, frontWheelsOffset, closestPointTreshold, controlGain);

    // load controller parameters
    float speedP, speedI, speedD, steerP, steerI, steerD;
    if(!m_handle.getParam("controller/speed/p", speedP))
        ROS_ERROR("Failed to get parameter \"/controller/speed/p\" from server\n");
    if(!m_handle.getParam("controller/speed/i", speedI))
        ROS_ERROR("Failed to get parameter \"/controller/speed/i\" from server\n");
    if(!m_handle.getParam("controller/speed/d", speedD))
        ROS_ERROR("Failed to get parameter \"/controller/speed/d\" from server\n");
    if(!m_handle.getParam("controller/steering/p", steerP))
        ROS_ERROR("Failed to get parameter \"/controller/steering/p\" from server\n");
    if(!m_handle.getParam("controller/steering/i", steerI))
        ROS_ERROR("Failed to get parameter \"/controller/steering/i\" from server\n");
    if(!m_handle.getParam("controller/steering/d", steerD))
        ROS_ERROR("Failed to get parameter \"/controller/steering/d\" from server\n");
    this->SetControllerParams(speedP, speedI, speedD, steerP, steerI, steerD);
}

void TrackingAlgorithm::FreshTrajectory()
{
    m_coneIndexOffset = 0;
}

void TrackingAlgorithm::SetPublisher(ros::Publisher targetPub)
{
    m_targetPub = targetPub;
}

void TrackingAlgorithm::SetParams(float carLength, float rearWheelsOffset, float frontWheelsOffset, float closestPointTreshold, float controlGain)
{
    m_carLength = carLength;
    m_rearWheelsOffset = rearWheelsOffset;
    m_frontWheelsOffset = frontWheelsOffset;
    m_closestPointTreshold = closestPointTreshold;
    m_controlGain = controlGain;
}

void TrackingAlgorithm::SetControllerParams(float speedP, float speedI, float speedD, float steerP, float steerI, float steerD)
{
    m_speedP = speedP;
    m_speedI = speedI;
    m_speedD = speedD;
    m_steeringP = steerP;
    m_steeringI = steerI;
    m_steeringD = steerD;
}

void TrackingAlgorithm::VisualizeTargetPoint(float p_x, float p_y)
{
    visualization_msgs::Marker marker;
    
    marker.color.r            = 1.0;
    marker.color.a            = 1.0;
    marker.pose.position.x    = p_x;
    marker.pose.position.y    = p_y;
    marker.pose.orientation.w = 1.0;
    marker.type               = visualization_msgs::Marker::SPHERE;
    marker.action             = visualization_msgs::Marker::ADD;
    marker.id                 = 0;
    marker.scale.x            = 0.5;
    marker.scale.y            = 0.5;
    marker.scale.z            = 0.5;
    marker.header.stamp       = ros::Time::now();
    marker.header.frame_id    = "map";
    m_targetPub.publish(marker);
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
    Control m_control;

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

    m_control.steeringAngle = ControlCommand(msg.speed);
    m_control.speed = msg.speed;

    return m_control;
}

void Stanley::ComputeFrontWheelPos(const sgtdv_msgs::CarPose::ConstPtr &carPose)
{
    const cv::Vec2f pos(carPose->position.x, carPose->position.y);
    m_frontWheelsPos = pos + cv::Vec2f(sinf(deg2rad(carPose->yaw)), cosf(deg2rad(carPose->yaw))) * m_frontWheelsOffset;   
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

        if (static_cast<float>(cv::norm(m_frontWheelsPos - point)) < m_closestPointTreshold)
        {
            m_coneIndexOffset++;

            continue;
        }

        VisualizeTargetPoint(point[0], point[1]);
        return point;
    }

    return cv::Vec2f(0.f, 0.f);
}

float Stanley::SpeedGain(float speed) const
{
    return m_controlGain / speed; //needs testing
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
    m_control.speed = msg.speed;
    
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
    sgtdv_msgs::Point2D next_point = msg.trajectory->points[nextIdx];
    VisualizeTargetPoint(next_point.x, next_point.y);
    
    const double theta = msg.carPose->yaw;
    //std::cout << "theta = " << theta << std::endl;
    const double alpha = std::atan2((next_point.y - msg.carPose->position.y),(next_point.x - msg.carPose->position.x)) - theta;
    //std::cout << "alpha = " << alpha << std::endl;
    const double lookAheadDist = std::hypot((next_point.y - msg.carPose->position.y), (next_point.x - msg.carPose->position.x));
    //std::cout << "look = " << lookAheadDist << std::endl;
    const double steeringAngleAct = m_control.steeringAngle;
    const double steeringAngleWish = std::atan2(2*std::sin(alpha)*m_carLength,lookAheadDist);
    const double steeringAngleError = steeringAngleWish - steeringAngleAct;
    //std::cout << "steering error = " << steeringAngleError << std::endl;

    m_control.steeringAngle = static_cast<float>(m_steeringP * steeringAngleError);

    return m_control;
}