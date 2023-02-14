/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/


#include <ros/ros.h>
#include "../include/PathTracking.h"
#include <sgtdv_msgs/Point2DArr.h>
#include <sgtdv_msgs/CarPose.h>
#include <sgtdv_msgs/CarVel.h>
#include "../include/Messages.h"

class PathTrackingSynch
{
public:
    PathTrackingSynch(ros::NodeHandle &handle);
    ~PathTrackingSynch();

    void SetPublishers(ros::Publisher cmdPub, ros::Publisher targetPub);
    void DoPlannedTrajectory(const sgtdv_msgs::Point2DArr::ConstPtr &msg);
    void DoPoseEstimate(const sgtdv_msgs::CarPose::ConstPtr &msg);
    void DoVelocityEstimate(const sgtdv_msgs::CarVel::ConstPtr &msg);
    void Do();

private:
    PathTracking m_pathTracking;
    PathTrackingMsg m_pathTrackingMsg;

    bool m_trajectoryReady = false;
    sgtdv_msgs::CarPose m_lastPose;

    // parameters
    float m_refSpeed;
    float m_constYawRate;
};