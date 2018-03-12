#ifndef LOS_GUIDANCE_LAW_H
#define LOS_GUIDANCE_LAW_H

#include <ros/ros.h>
#include <vector>
#include <array>
#include <cmath>
#include <limits>
#include <boost/numeric/odeint.hpp>

namespace los_guidance_law {

class LosGuidanceLaw
{
    friend std::array<double, 3> ComputeCourse(LosGuidanceLaw&, const double&, const double&, const double&,
                                          const double&, const double&, const double&, const double&);

public:
    // constructors
    LosGuidanceLaw(const std::vector<double> &course_state, const double &radius,
                   const double &step_size, const double &omega, const double &zeta);
    LosGuidanceLaw(): LosGuidanceLaw({0, 0, 0}, 1.0, 0.05, 0.4, 1.0) {}
    LosGuidanceLaw(const std::vector<double> &course_state): LosGuidanceLaw(course_state, 1.0, 0.05, 0.4, 1.0) {}
    LosGuidanceLaw(const std::vector<double> &course_state, const double &radius, const double &step_size):
        LosGuidanceLaw(course_state, radius, step_size, 0.4, 1.0) {}

private:
    // radius of the capture circle
    double radius_ = 1.0;

    // end time of the integration of the low-pass filter
    double step_size_ = 0.05;

    // los vector
    std::array<double, 2> los_point_;

    // desired course angle
    double course_angle_;

    // filtered course information
    std::vector<double> course_state_;

    // the reference model for trajectory generation
    double omega_;
    double zeta_;

public:
    void operator()(const std::vector<double> &x, std::vector<double> &dxdt, const double /* t */)
    {
        dxdt[0] = x[1];
        dxdt[1] = x[2];
        dxdt[2] = -pow(omega_,3)*x[0] - (2*zeta_+1)*pow(omega_,2)*x[1] - (2*zeta_+1)*omega_*x[2] + \
                pow(omega_,3)*course_angle_;
    }
};

// constructor definition
LosGuidanceLaw::LosGuidanceLaw(const std::vector<double> &course_state, const double &radius,
                               const double &step_size, const double &omega, const double &zeta):
        course_state_(course_state), radius_(radius), step_size_(step_size), omega_(omega), zeta_(zeta) {}

// define the member method used to compute the desired course state
// (x0, y0) is the previous waypoint, (x1, y1) is the waypoint to which the ship is pointed
std::array<double, 3> ComputeCourse(LosGuidanceLaw &law, const double &pos_x, const double &pos_y, const double &pos_heading,
                               const double &x0, const double &x1, const double &y0, const double &y1)
{


    bool feasible = true;
    std::array<double, 3> msg_course;
    msg_course[0] = law.course_state_[0];
    msg_course[1] = law.course_state_[1];
    msg_course[2] = law.course_state_[2];

    double dx = x1 - x0;
    double dy = y1 - y0;

    if (fabs(dx) > std::numeric_limits<double>::epsilon())
    {
        double x = pos_x;
        double y = pos_y;
        double d = dy/dx;
        double e = x0;
        double f = y0;
        double g = f - d*e;
        double a = 1 + pow(d,2);
        double b = 2*(d*g - d*y - x);
        double c = pow(x,2) + pow(y,2) + pow(g,2) - 2*g*y - pow(law.radius_,2);

        if (pow(b,2) - 4*a*c >= 0) {
            if (dx > 0) {
                law.los_point_[0] = (-b + sqrt(pow(b,2) - 4*a*c))/(2*a);
            }
            if (dx < 0) {
                law.los_point_[0] = (-b - sqrt(pow(b,2) - 4*a*c))/(2*a);
            }
            law.los_point_[1] = d*(law.los_point_[0] - x0) + y0;
        }
        else
        {
            feasible = false;
        }
    }
    else
    {
        if (pow(law.radius_,2) - pow((x1 - pos_x),2) >=0 )
        {
            law.los_point_[0] = x1;
            if (dy > 0) {
                law.los_point_[1] = pos_y + sqrt(pow(law.radius_,2) - pow((law.los_point_[0] - pos_x),2));
            }
            if (dy < 0) {
                law.los_point_[1] = pos_y - sqrt(pow(law.radius_,2) - pow((law.los_point_[0] - pos_x),2));
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
        law.course_angle_ = atan2(law.los_point_[1] - pos_y, law.los_point_[0] - pos_x);

        if (fabs(law.course_angle_ - pos_heading) > M_PI)
        {
            ROS_ERROR_THROTTLE(5, "This is the probelm I haven't solved perfectly");
            law.course_state_[0] = law.course_angle_;
            law.course_state_[1] = 0;
            law.course_state_[2] = 0;

            law.course_angle_ += boost::math::sign(pos_heading)*2*M_PI;
            msg_course[0] = pos_heading + (law.course_angle_ - pos_heading) * 0.1;
            msg_course[1] = (law.course_angle_ - pos_heading) * 0.1;
            msg_course[2] = 0;
        }
        else
        {
            // low-pass filter the obtained course angle, yielding rate and acceleration
            // the end time of the integration should be the publising rate of ship/pose
            size_t steps = boost::numeric::odeint::integrate(law, law.course_state_, 0.0, law.step_size_, law.step_size_/2);

            // publish the desired course message
            msg_course[0] = law.course_state_[0];
            msg_course[1] = law.course_state_[1];
            msg_course[2] = law.course_state_[2];
        }
    }
    else
    {
        // no course message published
        ROS_WARN_STREAM("There are no intersections");
    }

    return msg_course;
}

}

#endif // LOS_GUIDANCE_LAW_H
