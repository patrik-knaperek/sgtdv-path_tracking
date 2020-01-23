#include <ros/ros.h>
#include <chrono>

constexpr float FPS = 60.f;
constexpr float TIME_PER_FRAME = 1.f / FPS;

class PathTracking
{
public:
    PathTracking();
    ~PathTracking();

    void SetPublisher(ros::Publisher publisher);
    void Do();
private:
    ros::Publisher m_publisher;
};