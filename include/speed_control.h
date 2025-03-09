/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

#include "SGT_Macros.h"
#include "SGT_Utils.h"

constexpr float FPS = 60.f;
constexpr float TIME_PER_FRAME = 1.f / FPS;

class SpeedControl
{
public:
  struct Params
  {
    Utils::Range<double> cmd_range;
    int type;
    double a_limit, j_limit, K1, K2; // reference signal shaping parameters
    /* basic speed controller parameters */
    double p;
    double i;
    /* advanced speed controller parameters */
    double K, T;                     // vehicle 1st order model parameters
    double w0, b;                    // Pole Placement parameters
  };

public:
  explicit SpeedControl(const Params& params) : params_(params) {};

  ~SpeedControl() = default;

  int8_t computeSpeedCommand(const float act_speed, const float ref_speed);

  void reinit(void);

private:
  Params params_;

  double w_ = 0.;        // shaped reference signal (speed)
  double w_dot_ = 0.;    // 1st derivate of shaped reference signal (acceleration)
  double w_ddot_ = 0.;   // 2nd derivate of shaped reference signal (jerk)

  double jerk_integral_ = 0.;    // jerk integrator
  double acc_integral_ = 0.;     // acceleration integrator
  double aw_ = 0.;               // anti-windup feedback loop

  double speed_integral_ = 0.;   // IP controller integrator
};