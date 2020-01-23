#include "../include/PathTrackingSynch.h"

PathTrackingSynch::PathTrackingSynch()
{

}

PathTrackingSynch::~PathTrackingSynch()
{

}

void PathTrackingSynch::SetPublisher(ros::Publisher publisher)
{
    m_pathTracking.SetPublisher(publisher);
}