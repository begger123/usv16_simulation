#include "los_guidance_law.h"

#include <cmath>
#include <limits>

namespace guidance_law
{

// define the function used to compute the desired course state
// (x0, y0) is the previous waypoint, (x1, y1) is the waypoint to which the ship
// is pointed
std::array<double, 3> ComputeCourse(LosGuidanceLaw& law, const double& pos_x,
                                    const double& pos_y,
                                    const double& pos_heading, const double& x0,
                                    const double& x1, const double& y0,
                                    const double& y1)
{
  bool feasible = true;

  double dx = x1 - x0;
  double dy = y1 - y0;

  if (fabs(dx) > std::numeric_limits<double>::epsilon())
  {
    double x = pos_x;
    double y = pos_y;
    double d = dy / dx;
    double e = x0;
    double f = y0;
    double g = f - d * e;
    double a = 1 + pow(d, 2);
    double b = 2 * (d * g - d * y - x);
    double c =
        pow(x, 2) + pow(y, 2) + pow(g, 2) - 2 * g * y - pow(law.radius_, 2);

    if (pow(b, 2) - 4 * a * c >= 0)
    {
      if (dx > 0)
      {
        law.los_point_[0] = (-b + sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
      }
      if (dx < 0)
      {
        law.los_point_[0] = (-b - sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
      }
      law.los_point_[1] = d * (law.los_point_[0] - x0) + y0;
    }
    else
    {
      feasible = false;
    }
  }
  else
  {
    if (pow(law.radius_, 2) - pow((x1 - pos_x), 2) >= 0)
    {
      law.los_point_[0] = x1;
      if (dy > 0)
      {
        law.los_point_[1] = pos_y + sqrt(pow(law.radius_, 2) -
                                         pow((law.los_point_[0] - pos_x), 2));
      }
      if (dy < 0)
      {
        law.los_point_[1] = pos_y - sqrt(pow(law.radius_, 2) -
                                         pow((law.los_point_[0] - pos_x), 2));
      }
    }
    else
    {
      feasible = false;
    }
  }

  // compute the desired course angle using the intersection point
  if (feasible)
  {
    law.course_angle_ =
        atan2(law.los_point_[1] - pos_y, law.los_point_[0] - pos_x);
  }
  else
  {
    ROS_DEBUG_THROTTLE(5, "The los guidance law is not feasible. Head for the "
                          "desired waypoint instead.");
    law.course_angle_ = atan2(y1 - pos_y, x1 - pos_x);
  }

  if (fabs(law.course_angle_ - pos_heading) > M_PI)
  {
    law.course_angle_ += boost::math::sign(pos_heading) * 2 * M_PI;
  }

  // publish the desired course message
  std::array<double, 3> msg_course;
  msg_course[0] = law.course_angle_;
  msg_course[1] = 0;
  msg_course[2] = 0;

  return msg_course;
}
}
