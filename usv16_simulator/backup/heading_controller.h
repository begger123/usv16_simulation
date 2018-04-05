#ifndef HEADING_CONTROLLER_H
#define HEADING_CONTROLLER_H

#include <vector>
#include <cmath>
#include <boost/numeric/odeint.hpp>
#include "ship.h"

class HeadingController
{
    friend std::vector<double> ComputeActuation(HeadingController&, const double&, const double&, const double&, const double&,
                                                const double&, const double&, const double&, const double&);

public:
    // constructor
    HeadingController(const Ship&, const double&, const double&, const double&, const double&);
    HeadingController(const Ship& vehicle): HeadingController(vehicle, 0.75, 25, 10, 2.5) {}

private:
    double m11_, m22_, n11_, n22_, m32_, m33_, n32_, n33_, arm_;

    // controller parameters
    const double c_, k1_, k2_, k3_;
    // surge speed setpoint
    const double ud_ = 0.1;

public:
    // error variables
    double z1_, z21_, z22_, z23_;
    // alpha2 in the control law
    double alpha2_;
    // alpha2_model input
    double alpha2_input_;

    void operator()(const std::vector<double>&, std::vector<double>&, const double);
    std::vector<double> ComputeActuation(const double&, const double&, const double&, const double&,
                                         const double&, const double&, const double&, const double&);

};

// constructor
HeadingController::HeadingController(const Ship &vehicle, const double &k0, const double &k1, const double &k2,
                               const double &k3): c_(k0), k1_(k1), k2_(k2), k3_(k3) {
    m11_ = vehicle.m11_;
    m22_ = vehicle.m22_;
    n11_ = vehicle.n11_;
    n22_ = vehicle.n22_;
    m32_ = vehicle.m23_;
    m33_ = vehicle.m33_;
    n32_ = vehicle.n23_;
    n33_ = vehicle.n33_;
    arm_ = vehicle.arm_;
}

void HeadingController::operator()(const std::vector<double> &x, std::vector<double> &dxdt, const double /* t */)
{
    dxdt[0] = (alpha2_input_ - k2_ * x[0])/(m22_ - m32_/arm_);
}

// the function used to compute the actuations
std::vector<double> ComputeActuation(HeadingController &controller, const double &u, const double &v, const double &r, const double &psi,
                                     const double &psi_d, const double &r_d, const double &dr_d, const double &step_size)
{
    std::vector<double> actuation(3, 0);

    double z1 = psi - psi_d;
    double alpha3 = -controller.c_ * z1 + r_d;
    double dalpha3 = -controller.c_ * (r - r_d) + dr_d;

    std::vector<double> x = {controller.alpha2_};
    controller.alpha2_input_ = -(controller.m32_-controller.m33_/controller.arm_)*dalpha3 -\
            (controller.n22_-controller.n32_/controller.arm_-controller.k2_)*v - \
            (controller.n32_-controller.n33_/controller.arm_)*r - (controller.k3_*(r-alpha3)+z1)/controller.arm_;
    double dalpha2 = (controller.alpha2_input_ - controller.k2_*controller.alpha2_)/(controller.m22_ - controller.m32_/controller.arm_);
    size_t steps = boost::numeric::odeint::integrate(controller, x, 0.0, step_size, step_size/5);

    // compute the actuation
    actuation[0] = controller.n11_*u - controller.k1_*(u-controller.ud_);
    actuation[2] = controller.m32_*dalpha2 + controller.m33_*dalpha3 +
            controller.n32_*v + controller.n33_*r - controller.k3_*(r-alpha3) - z1;

    controller.alpha2_ = x[0];
    controller.z1_ = z1;
    controller.z21_ = u - controller.ud_;
    controller.z22_ = v - controller.alpha2_;
    controller.z23_ = r - alpha3;

    // set saturation
    actuation[0] = (actuation[0] > 2) ? 2 : actuation[0];
    actuation[2] = (actuation[2] > 0.5) ? 0.5 : actuation[2];
    actuation[2] = (actuation[2] < -0.5) ? -0.5 : actuation[2];
    actuation[1] = -actuation[2]/controller.arm_;

    return actuation;
}

#endif // HEADING_CONTROLLER_H
