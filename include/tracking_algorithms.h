/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Tereza Ábelová, Juraj Krasňanský, Patrik Knaperek
/*****************************************************/

#pragma once

/* C++ */
#include <cmath>
#include <Eigen/Eigen>

/* ROS */
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

/* SGT */
#include <sgtdv_msgs/Point2D.h>
#include <sgtdv_msgs/Point2DArr.h>
#include <sgtdv_msgs/CarPose.h>
#include <sgtdv_msgs/CarVel.h>
#include <sgtdv_msgs/Control.h>
#include "../../SGT_Macros.h"
#include "../include/messages.h"

constexpr float FPS = 60.f;
constexpr float TIME_PER_FRAME = 1.f / FPS;

#define deg2rad(x) (x*M_PI/180.f)
#define rad2deg(x) (x*180.f/M_PI)

class TrackingAlgorithm
{
public:
  struct Params
  {
    /* vehicle parameters */
    float car_length;
    float rear_wheels_offset;
    float front_wheels_offset;

    /* speed PID controller parameters */
    // float ref_speed;
    float speed_p;
    float speed_i;
    float speed_min;
    float speed_max;
    float speed_raise_rate;

    /* steering control parameters*/
    float steering_k;
    float steering_min;
    float steering_max;
    float lookahead_dist_min;
    float lookahead_dist_max;

    bool track_loop;
  };

  virtual void update(const PathTrackingMsg &msg, sgtdv_msgs::ControlPtr &control_msg) = 0;
  virtual void setParams(const Params &params)
  {
    params_ = params;
  };
  
  virtual void setRefSpeed(const float ref_speed)
  {
    ref_speed_ = ref_speed;
  };

  void resetIntegral()
  {
    speed_integral_error_ = 0.;
  };

protected:
  TrackingAlgorithm(const ros::NodeHandle &handle
  #ifdef SGT_VISUALIZATION
    , const ros::Publisher& target_pub
    , const ros::Publisher& steering_pose_pub
  #endif /* SGT_VISUALIZATION */
  );
  ~TrackingAlgorithm() = default;

  virtual int8_t computeSpeedCommand(const float act_speed, const int8_t speed_cmd_prev);
  size_t findClosestPointIdx(const sgtdv_msgs::Point2DArr::ConstPtr &trajectory, 
                            const Eigen::Ref<const Eigen::Vector2f>& pos) const;

#ifdef SGT_VISUALIZATION
  virtual void visualizePoint(const Eigen::Ref<const Eigen::Vector2f>& point, const int point_id, 
                              const std::string& ns, const Eigen::Ref<const Eigen::Vector3f> color) const;
  virtual void visualizeSteering() const;

  ros::Publisher target_pub_;
  ros::Publisher steering_pose_pub_;
#endif /* SGT_VISUALIZATION */

  Control control_;
  Params params_;

private:
  float ref_speed_ = 0.;
  float speed_integral_error_ = 0.;

};

class PurePursuit : public TrackingAlgorithm
{
public:
  PurePursuit(const ros::NodeHandle &handle
  #ifdef SGT_VISUALIZATION
    , const ros::Publisher& target_pub
    , const ros::Publisher& steering_pose_pub
  #endif /* SGT_VISUALIZATION */
  );
  ~PurePursuit() = default;

  void update(const PathTrackingMsg &msg, sgtdv_msgs::ControlPtr &control_msg) override;

private:
  Eigen::Vector2f rear_wheels_pos_;
  float lookahead_dist_;

  void computeRearWheelPos(const sgtdv_msgs::CarPose::ConstPtr &car_pose);
  void computeLookAheadDist(const float speed);
  Eigen::Vector2f findTargetPoint(const sgtdv_msgs::Point2DArr::ConstPtr &trajectory) const;
  float computeSteeringCommand(const PathTrackingMsg &msg, const Eigen::Ref<const Eigen::Vector2f>& target_point);
};
