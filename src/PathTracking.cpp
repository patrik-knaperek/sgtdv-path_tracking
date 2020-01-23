#include "../include/PathTracking.h"

PathTracking::PathTracking()
{

}

PathTracking::~PathTracking()
{

}

void PathTracking::SetPublisher(ros::Publisher publisher)
{
    m_publisher = publisher;
}