/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

#pragma once

#include "SGT_Utils.h"
#include "SGT_Macros.h"
#include <sgtdv_msgs/PathTrackingMsg.h>

class SteeringControl
{
public:
  struct Params 
  {
    /* vehicle parameters */
    float car_length;
    float rear_wheels_offset;
    float front_wheels_offset;

    Utils::Range<float> range;

    /* Pure Pursuit steering control parameters */
    float k;
    float smooth;
    Utils::Range<float> lookahead_dist;

    bool track_loop;
  };
public:
  explicit SteeringControl(const Params& params): params_(params) {};
  
  ~SteeringControl() = default;

  float computeSteeringCommand(const sgtdv_msgs::PathTrackingMsg &msg);

  
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
  float computeLookAheadDist(const float speed) const;

  Eigen::Vector2f findTargetPoint(const sgtdv_msgs::Point2DArr &trajectory,
                                  const Eigen::Vector2f &rear_axle_pos,
                                  const float lookahead_dist);

  Eigen::Vector2f computeRearAxlePos(const sgtdv_msgs::CarPose &car_pose);

  Params params_;

  float cmd_last_ = 0.f;

#ifdef SGT_VISUALIZATION
  std::pair<Eigen::Vector2f, Eigen::Vector2f> pure_pursuit_points_;
  std::pair<Eigen::Vector2f, Eigen::Vector2f> axle_pos_;
#endif /* SGT_VISUALIZATION */
};