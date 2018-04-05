#ifndef THRUST_ALLOCATION_H
#define THRUST_ALLOCATION_H

#include <array>
#include <sstream>
#include "usv16/control.h"
#include "Array.h"

typedef struct
{

  double T_fwd;
  double T_rev;
  double lx;
  double ly;

} thrust_allocaton_params;

class ThrustAllocation
{
public:
  /////////////////
  // Constructor //
  /////////////////

  ThrustAllocation() = default;
  ThrustAllocation(const double& T_fwd, const double& T_rev, const double& lx,
                   const double& ly);

  ////////////////////
  // Public Methods //
  ////////////////////

  std::array<double, 7> ComputeOutput(const std::array<double, 3>& input);

private:
  ////////////////////
  // Private Fields //
  ////////////////////

  // maximum actuation when forward
  double T_fwd_ = 0;

  // maximum actuation when reverse
  double T_rev_ = 0;

  // the coordinate of the azimuth thruster in the body-fixed frame
  double lx_ = 0;
  double ly_ = 0;

  // variables for QuadProg++
  Matrix<double> G_, CE_;
  Matrix<double> CI_1_, CI_2_, CI_3_, CI_4_;
  Vector<double> g0_, x_;
  Vector<double> ce0_;
  Vector<double> ci0_1_, ci0_2_, ci0_3_, ci0_4_;
  int n_, m_, p_;
};

#endif // THRUST_ALLOCATION_H
