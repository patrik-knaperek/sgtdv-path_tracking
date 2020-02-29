#include <ros/ros.h>
#include "../include/PathTracking.h"
#include <sgtdv_msgs/Point2DArr.h>
#include <sgtdv_msgs/CarState.h>
#include "../include/Messages.h"
//#include <IMU_MSG>

constexpr float CONST_SPEED = 6.f;
constexpr float CONST_YAW_RATE = 10.f;

class PathTrackingSynch
{
public:
    PathTrackingSynch();
    ~PathTrackingSynch();

    void SetPublisher(ros::Publisher publisher);
    void DoPlannedTrajectory(const sgtdv_msgs::Point2DArr::ConstPtr &msg);
    void DoPoseEstimate(const sgtdv_msgs::CarState::ConstPtr &msg);
    void Do();

private:
    PathTracking m_pathTracking;
    PathTrackingMsg m_pathTrackingMsg;
};