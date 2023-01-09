/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/


#include <ros/ros.h>
#include <chrono>
#include <sgtdv_msgs/PathTrackingMsg.h>
#include "../include/Messages.h"
#include "../include/TrackingAlgorithms.h"
#include <sgtdv_msgs/Control.h>

constexpr float FPS = 60.f;
constexpr float TIME_PER_FRAME = 1.f / FPS;

class PathTracking
{
public:
    PathTracking(ros::NodeHandle &handle);
    ~PathTracking();

    void SetPublishers(ros::Publisher cmdPub, ros::Publisher targetPub);
    void Do(const PathTrackingMsg &msg);
    void FreshTrajectory();
    void SetParams(float carLength, float rearWheelsOffset, float frontWheelsOffset, float closestPointTreshold, float controlGain)
    {
        m_algorithm->SetParams(carLength, rearWheelsOffset, frontWheelsOffset, closestPointTreshold, controlGain);
    };
    void SetControllerParams(float speedP, float speedI, float speedD, float steerP, float steerI, float steerD)
    {
        m_algorithm->SetControllerParams(speedP, speedI, speedD, steerP, steerI, steerD);
    };
    
private:
    ros::Publisher m_publisher;
    TrackingAlgorithm *m_algorithm = nullptr;

    void HandleAlgorithmResult(sgtdv_msgs::ControlPtr &msg, const Control &result);
};