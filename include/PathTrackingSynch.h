/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/


#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include "../include/PathTracking.h"
#include <sgtdv_msgs/Point2DArr.h>
#include <sgtdv_msgs/CarPose.h>
#include <sgtdv_msgs/CarVel.h>
#include "../include/Messages.h"

class PathTrackingSynch
{
public:
    PathTrackingSynch(const ros::NodeHandle &handle);
    ~PathTrackingSynch() = default;

    void SetCmdPublisher(const ros::Publisher &cmdPub)
    {
        m_pathTracking.SetCmdPublisher(cmdPub);
    };        
    
    void SetVisualizationPublishers(const ros::Publisher &targetPub, const ros::Publisher &steeringPosePub)
    {
        m_pathTracking.SetVisualizationPublishers(targetPub, steeringPosePub);
    };
    
    void DoPlannedTrajectory(const sgtdv_msgs::Point2DArr::ConstPtr &msg);
    void DoPoseEstimate(const sgtdv_msgs::CarPose::ConstPtr &msg);
    void DoVelocityEstimate(const sgtdv_msgs::CarVel::ConstPtr &msg);
    void StopCallback(const std_msgs::Empty::ConstPtr &msg);
    void StartCallback(const std_msgs::Empty::ConstPtr &msg);
    void Do();

private:
    PathTracking m_pathTracking;
    PathTrackingMsg m_pathTrackingMsg;

    bool m_trajectoryReady;
    bool m_poseReady;
    bool m_velocityReady;
    sgtdv_msgs::CarPose m_lastPose;

    // parameters
    float m_refSpeed;
    float m_constYawRate;
};