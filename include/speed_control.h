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
    /* speed controller parameters */
    float p;
    float i;
    Utils::Range<float> range;
    float a_limit, j_limit, K1, K2; // reference signal shaping parameters
    float k1, k2;                   // feed-forward gains
  };

public:
  explicit SpeedControl(const Params& params) : params_(params) {};

  ~SpeedControl() = default;

  int8_t computeSpeedCommand(const float act_speed, const float ref_speed);

private:
  Params params_;

  float speed_integral_error_ = 0.;
  int8_t cmd_last_ = 0;
};