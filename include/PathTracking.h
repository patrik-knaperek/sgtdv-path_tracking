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
#include <geometry_msgs/PoseStamped.h>

class PathTracking
{
public:
    PathTracking(const ros::NodeHandle &handle);
    ~PathTracking() = default;

    void LoadParams(const ros::NodeHandle &handle) const;
    void SetCmdPublisher(const ros::Publisher &cmdPub)
    {
        m_cmdPublisher = cmdPub;
    };
    void SetVisualizationPublishers(const ros::Publisher &targetPub, const ros::Publisher &steeringPosePub)
    {
        m_algorithm->SetVisualizationPublishers(targetPub, steeringPosePub);
    };
    void Do(const PathTrackingMsg &msg);
    void StopVehicle();
    void StartVehicle();
    void SetRefSpeed(const float refSpeed)
    {
        m_algorithm->SetRefSpeed(refSpeed);
    }
private:
    template<typename T> void GetParam(const ros::NodeHandle &handle, const std::string &name, T* storage) const;
    template<typename T> void GetParam(const ros::NodeHandle &handle, const std::string &name,
                                        const T &defaultValue, T* storage) const;
    ros::Publisher m_cmdPublisher;
    TrackingAlgorithm *m_algorithm;
    bool m_stopped;
};