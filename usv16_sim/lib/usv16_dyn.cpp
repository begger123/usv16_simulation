#include "usv16_sim/usv16_dyn.h"

void usv16_dyn(const double tau[3], const double vel[3], double output[3])
{
  double U;
  double dv0[9];
  double dv1[9];
  double dv2[9];
  double Xuu[9];
  double b_vel;
  double dv3[9];
  int i0;
  double B[3];
  static const short iv0[3] = { 0, -300, 40 };

  double d0;
  int i1;
  U = sqrt(vel[0] * vel[0] + vel[1] * vel[1]);
  dv0[0] = 0.0;
  dv0[3] = 0.0;
  dv0[6] = -180.0 * vel[1];
  dv0[1] = 0.0;
  dv0[4] = 0.0;
  dv0[7] = -180.0 * (0.0 - vel[0]);
  dv0[2] = 180.0 * vel[1];
  dv0[5] = 180.0 * (0.0 - vel[0]);
  dv0[8] = 0.0;
  dv1[0] = 0.0;
  dv1[3] = 0.0;
  dv1[6] = 50.0 * vel[2] + -37.0 * vel[1];
  dv1[1] = 0.0;
  dv1[4] = 0.0;
  dv1[7] = 27.0 * vel[0];
  dv1[2] = -(50.0 * vel[2] + -37.0 * vel[1]);
  dv1[5] = -27.0 * vel[0];
  dv1[8] = 0.0;
  dv2[2] = 0.0;
  dv2[5] = 40.0;
  if (U < 0.3) {
    dv2[8] = -400.0;
  } else {
    dv2[8] = -(600.0 * U + 200.0);
  }

  if (vel[0] > 0.0) {
    b_vel = -112.4;
  } else {
    b_vel = -198.1;
  }

  Xuu[0] = b_vel * fabs(vel[0]);
  Xuu[3] = 0.0;
  Xuu[6] = 0.0;
  for (i0 = 0; i0 < 3; i0++) {
    dv2[3 * i0] = 0.0;
    dv2[1 + 3 * i0] = iv0[i0];
    Xuu[1 + 3 * i0] = 0.0;
    Xuu[2 + 3 * i0] = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      dv3[i1 + 3 * i0] = -(((dv0[i1 + 3 * i0] + dv1[i1 + 3 * i0]) + -dv2[i1 + 3 *
                            i0]) + -Xuu[i1 + 3 * i0]);
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    d0 = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      d0 += dv3[i0 + 3 * i1] * vel[i1];
    }

    B[i0] = d0 + tau[i0];
  }

  output[2] = (B[2] - B[1] * -0.2304147465437788) / 332.24080644015578;
  output[1] = B[1] - output[2] * -50.0;
  output[1] /= 217.0;
  output[0] = B[0] / 207.0;
}

void usv16_dyn_initialize()
{
}

void usv16_dyn_terminate()
{
}
