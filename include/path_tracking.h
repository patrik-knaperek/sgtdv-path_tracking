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

#include "steering_control.h"
#include "speed_control.h"

class PathTracking
{
public:
  struct Params
  {
    SpeedControl::Params speed;
    SteeringControl::Params steering;
  };

public:
  PathTracking() = default;
  ~PathTracking() = default;

  void init(const Params& params);

  void update(const sgtdv_msgs::PathTrackingMsg &msg, sgtdv_msgs::Control *cmd);

  void stopVehicle(void);

  void startVehicle(void);

#ifdef SGT_VISUALIZATION
  std::pair<Eigen::Vector2f, Eigen::Vector2f> getPurePursuitPoints(void) const
  {
    return steering_control_obj_->getPurePursuitPoints();
  };

  std::pair<Eigen::Vector2f, Eigen::Vector2f> getAxlePositions(void) const
  {
    return steering_control_obj_->getAxlePositions();
  };
#endif /* SGT_VISUALIZATION */


private:

  SpeedControl *speed_control_obj_ = nullptr;
  SteeringControl *steering_control_obj_ = nullptr;
  
  bool stopped_ = true;
};
