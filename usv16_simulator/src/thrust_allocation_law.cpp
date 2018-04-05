#include "thrust_allocation_law.h"

#include <array>
#include <cmath>
#include <algorithm>
#include <sstream>
#include "QuadProg++.h"

using namespace std;

/////////////////
// Constructor //
/////////////////

ThrustAllocation::ThrustAllocation(const double& T_fwd, const double& T_rev,
                                   const double& lx, const double& ly)
    : T_fwd_(T_fwd), T_rev_(T_rev), lx_(lx), ly_(ly)
{
  char ch;

  n_ = 7;
  x_.resize(n_);
  G_.resize(n_, n_);
  {
    istringstream is("1, 0, 0, 0, 0, 0, 0, "
                     "0, 1, 0, 0, 0, 0, 0, "
                     "0, 0, 1, 0, 0, 0, 0, "
                     "0, 0, 0, 1, 0, 0, 0, "
                     "0, 0, 0, 0, 1000, 0, 0, "
                     "0, 0, 0, 0, 0, 1000, 0, "
                     "0, 0, 0, 0, 0, 0, 1000  ");

    for (int i = 0; i < n_; i++)
      for (int j = 0; j < n_; j++)
        is >> G_[i][j] >> ch;
  }

  g0_.resize(n_);
  {
    istringstream is("0, 0, 0, 0, 0, 0, 0 ");

    for (int i = 0; i < n_; i++)
      is >> g0_[i] >> ch;
  }

  m_ = 3;
  ce0_.resize(m_);
  CE_.resize(n_, m_);
  {
    istringstream is("1, 0, 1.2, "
                     "0, 1, -2.4, "
                     "1, 0, -1.2, "
                     "0, 1, -2.4, "
                     "1, 0, 0, "
                     "0, 1, 0, "
                     "0, 0, 1");

    for (int i = 0; i < n_; i++)
      for (int j = 0; j < m_; j++)
        is >> CE_[i][j] >> ch;
  }
  CE_[0][2] = ly;
  CE_[1][2] = -lx;
  CE_[2][2] = -ly;
  CE_[3][2] = -lx;

  p_ = 8;
  // case 1: both forward
  CI_1_.resize(n_, p_);
  {
    istringstream is("1, 1, -1, -1, 0, 0, 0, 0,"
                     "-1, 1, -0.4142, 0.4142, 0, 0, 0, 0,"
                     "0, 0, 0, 0, 1, 1, -1, -1,"
                     "0, 0, 0, 0, -1, 1, -0.4142, 0.4142,"
                     "0, 0, 0, 0, 0, 0, 0, 0, "
                     "0, 0, 0, 0, 0, 0, 0, 0, "
                     "0, 0, 0, 0, 0, 0, 0, 0 ");

    for (int i = 0; i < n_; i++)
      for (int j = 0; j < p_; j++)
        is >> CI_1_[i][j] >> ch;
  }

  ci0_1_.resize(p_);
  {
    istringstream is("0, 0, 120, 120, 0, 0, 120, 120 ");

    for (int j = 0; j < p_; j++)
      is >> ci0_1_[j] >> ch;
  }
  ci0_1_[2] = T_fwd;
  ci0_1_[3] = T_fwd;
  ci0_1_[6] = T_fwd;
  ci0_1_[7] = T_fwd;

  // case 2: both backward
  CI_2_.resize(n_, p_);
  {
    istringstream is("-1, -1, 1, 1, 0, 0, 0, 0,"
                     "1, -1, -0.4142, 0.4142, 0, 0, 0, 0,"
                     "0, 0, 0, 0, -1, -1, 1, 1,"
                     "0, 0, 0, 0, 1, -1, -0.4142, 0.4142,"
                     "0, 0, 0, 0, 0, 0, 0, 0, "
                     "0, 0, 0, 0, 0, 0, 0, 0, "
                     "0, 0, 0, 0, 0, 0, 0, 0 ");

    for (int i = 0; i < n_; i++)
      for (int j = 0; j < p_; j++)
        is >> CI_2_[i][j] >> ch;
  }

  ci0_2_.resize(p_);
  {
    istringstream is("0, 0, 100, 100, 0, 0, 100, 100 ");

    for (int j = 0; j < p_; j++)
      is >> ci0_2_[j] >> ch;
  }
  ci0_2_[2] = T_rev;
  ci0_2_[3] = T_rev;
  ci0_2_[6] = T_rev;
  ci0_2_[7] = T_rev;

  // case 3: left forward, right backward
  CI_3_.resize(n_, p_);
  {
    istringstream is("1, 1, -1, -1, 0, 0, 0, 0,"
                     "-1, 1, -0.4142, 0.4142, 0, 0, 0, 0,"
                     "0, 0, 0, 0, -1, -1, 1, 1,"
                     "0, 0, 0, 0, 1, -1, -0.4142, 0.4142,"
                     "0, 0, 0, 0, 0, 0, 0, 0, "
                     "0, 0, 0, 0, 0, 0, 0, 0, "
                     "0, 0, 0, 0, 0, 0, 0, 0 ");

    for (int i = 0; i < n_; i++)
      for (int j = 0; j < p_; j++)
        is >> CI_3_[i][j] >> ch;
  }

  ci0_3_.resize(p_);
  {
    istringstream is("0, 0, 120, 120, 0, 0, 100, 100 ");

    for (int j = 0; j < p_; j++)
      is >> ci0_3_[j] >> ch;
  }
  ci0_3_[2] = T_fwd;
  ci0_3_[3] = T_fwd;
  ci0_3_[6] = T_rev;
  ci0_3_[7] = T_rev;

  // case 4: left backward, right forward
  CI_4_.resize(n_, p_);
  {
    istringstream is("-1, -1, 1, 1, 0, 0, 0, 0,"
                     "1, -1, -0.4142, 0.4142, 0, 0, 0, 0,"
                     "0, 0, 0, 0, 1, 1, -1, -1,"
                     "0, 0, 0, 0, -1, 1, -0.4142, 0.4142,"
                     "0, 0, 0, 0, 0, 0, 0, 0, "
                     "0, 0, 0, 0, 0, 0, 0, 0, "
                     "0, 0, 0, 0, 0, 0, 0, 0 ");

    for (int i = 0; i < n_; i++)
      for (int j = 0; j < p_; j++)
        is >> CI_4_[i][j] >> ch;
  }

  ci0_4_.resize(p_);
  {
    istringstream is("0, 0, 100, 100, 0, 0, 120, 120 ");

    for (int j = 0; j < p_; j++)
      is >> ci0_4_[j] >> ch;
  }
  ci0_4_[2] = T_rev;
  ci0_4_[3] = T_rev;
  ci0_4_[6] = T_fwd;
  ci0_4_[7] = T_fwd;
}

array<double, 7> ThrustAllocation::ComputeOutput(const array<double, 3>& input)
{

  ce0_[0] = -input[0];
  ce0_[1] = -input[1];
  ce0_[2] = -input[2];

  // define the output
  // the last three elements stand for the real actuation
  array<double, 7> output = {0, 0, 0, 0, input[0], input[1], input[2]};

  if (ce0_[1] != 0) // station-keeping mode
  {
    array<Vector<double>, 4> x = {x_, x_, x_, x_};
    array<double, 4> f;
    f[0] = solve_quadprog(G_, g0_, CE_, ce0_, CI_1_, ci0_1_, x[0]);
    f[1] = solve_quadprog(G_, g0_, CE_, ce0_, CI_2_, ci0_2_, x[1]);
    f[2] = solve_quadprog(G_, g0_, CE_, ce0_, CI_3_, ci0_3_, x[2]);
    f[3] = solve_quadprog(G_, g0_, CE_, ce0_, CI_4_, ci0_4_, x[3]);

    // find the smallest function value (best allocation strategy)
    array<double, 4>::const_iterator result = min_element(f.cbegin(), f.cend());
    auto index = distance(f.cbegin(), result);

    if (index != 4)
    {
      auto temp = x[index];
      for (int j = 0; j < 4; ++j)
      {
        output[j] = temp[j];
      }
    }
  }
  else // path-following mode
  {
    output[0] = input[0] / 2 + input[2] / 2 / ly_;
    output[1] = 0;
    output[2] = input[0] / 2 - input[2] / 2 / ly_;
    output[3] = 0;

    // set the saturation
    output[0] = (output[0] > T_fwd_) ? T_fwd_ : output[0];
    output[0] = (output[0] < -T_rev_) ? -T_rev_ : output[0];
    output[2] = (output[2] > T_fwd_) ? T_fwd_ : output[2];
    output[2] = (output[2] < -T_rev_) ? -T_fwd_ : output[2];
  }

  // compute the real acutation
  output[4] = output[0] + output[2];
  output[5] = output[1] + output[3];
  output[6] = (output[0] - output[2]) * ly_ - (output[1] + output[3]) * lx_;

  return output;
}
