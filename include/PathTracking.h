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
    PathTracking();
    ~PathTracking();

    void SetPublisher(ros::Publisher publisher);
    void Do(const PathTrackingMsg &msg);
private:
    ros::Publisher m_publisher;
    TrackingAlgorithm *m_algorithm = nullptr;

    void HandleAlgorithmResult(sgtdv_msgs::ControlPtr &msg, const Control &result);
};