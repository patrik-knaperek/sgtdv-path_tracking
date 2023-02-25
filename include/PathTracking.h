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

class PathTracking
{
public:
    PathTracking(ros::NodeHandle &handle);
    ~PathTracking();

    void LoadParams(ros::NodeHandle &handle);
    void SetPublishers(ros::Publisher cmdPub, ros::Publisher targetPub);
    void Do(const PathTrackingMsg &msg);
    //void FreshTrajectory();
    
private:
    ros::Publisher m_publisher;
    TrackingAlgorithm *m_algorithm = nullptr;

    void HandleAlgorithmResult(sgtdv_msgs::ControlPtr &msg, const Control &result);
};