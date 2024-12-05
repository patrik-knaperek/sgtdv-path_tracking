/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský, Patrik Knaperek
/*****************************************************/

#pragma once

/* C++ */
#include <Eigen/Eigen>

/* SGT-DV */
#include <sgtdv_msgs/Control.h>
#include <sgtdv_msgs/PathTrackingMsg.h>
#include "SGT_Macros.h"
#include "SGT_Utils.h"

constexpr float FPS = 60.f;
constexpr float TIME_PER_FRAME = 1.f / FPS;

#define deg2rad(x) (x*M_PI/180.f)
#define rad2deg(x) (x*180.f/M_PI)

class PathTracking
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
    Utils::Range<float> speed;
    float speed_raise_rate;

    /* steering control parameters*/
    float steering_k;
    float steering_smooth;
    Utils::Range<float> steering;
    Utils::Range<float> lookahead_dist;

    bool track_loop;
  };

public:
  PathTracking() = default;
  ~PathTracking() = default;

  void update(const sgtdv_msgs::PathTrackingMsg &msg, sgtdv_msgs::Control *cmd);

  void resetIntegral()
  {
    speed_integral_error_ = 0.;
  };

  /* Setters & Getters*/
  void setParams(const Params &params)
  {
    params_ = params;
  };
  
  void setRefSpeed(const float ref_speed)
  {
    ref_speed_ = ref_speed;
  };

#ifdef SGT_VISUALIZATION
  std::pair<Eigen::Vector2f, Eigen::Vector2f> getPurePursuitPoints(void) const
  {
    return pure_pursuit_points_;
  };

  std::pair<Eigen::Vector2f, Eigen::Vector2f> getAxlePositions(void) const
  {
    return axle_pos_;
  };
#endif /* SGT_VISUALIZATION */


private:
  int8_t computeSpeedCommand(const float act_speed, const int8_t speed_cmd_prev);

  float computeSteeringCommand(const sgtdv_msgs::PathTrackingMsg &msg, const float steer_cmd_prev);

  float computeLookAheadDist(const float speed) const;

  Eigen::Vector2f findTargetPoint(const sgtdv_msgs::Point2DArr &trajectory,
                                  const Eigen::Vector2f &rear_axle_pos,
                                  const float lookahead_dist);

  size_t findClosestPointIdx(const sgtdv_msgs::Point2DArr &trajectory, 
                            const Eigen::Vector2f& pos) const;

  Eigen::Vector2f computeRearAxlePos(const sgtdv_msgs::CarPose &car_pose);

  Params params_;

  float ref_speed_ = 0.;
  float speed_integral_error_ = 0.;

#ifdef SGT_VISUALIZATION
    std::pair<Eigen::Vector2f, Eigen::Vector2f> pure_pursuit_points_;
    std::pair<Eigen::Vector2f, Eigen::Vector2f> axle_pos_;
#endif /* SGT_VISUALIZATION */
};
