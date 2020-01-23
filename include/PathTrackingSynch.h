#include <ros/ros.h>
#include "../include/PathTracking.h"

class PathTrackingSynch
{
public:
    PathTrackingSynch();
    ~PathTrackingSynch();

    void SetPublisher(ros::Publisher publisher);
    
private:
    PathTracking m_pathTracking;
};