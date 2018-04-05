#ifndef LOS_GUIDANCE_LAW_H
#define LOS_GUIDANCE_LAW_H

#include <ros/ros.h>
#include <vector>
#include <array>

namespace guidance_law
{

/////////////////////////
// Forward Declaration //
/////////////////////////

class LosGuidanceLaw;
std::array<double, 3> ComputeCourse(LosGuidanceLaw&, const double&,
                                    const double&, const double&, const double&,
                                    const double&, const double&,
                                    const double&);

class LosGuidanceLaw
{
  friend std::array<double, 3> ComputeCourse(LosGuidanceLaw&, const double&,
                                             const double&, const double&,
                                             const double&, const double&,
                                             const double&, const double&);

public:
  /////////////////
  // Constructor //
  /////////////////

  LosGuidanceLaw() = default;
  LosGuidanceLaw(const double& radius) : radius_(radius) {}

private:
  ////////////////////
  // Private Fields //
  ////////////////////

  // radius of the circle
  double radius_;

  // los vector
  std::array<double, 2> los_point_;

  // desired course angle
  double course_angle_;
};
}

#endif // LOS_GUIDANCE_LAW_H
