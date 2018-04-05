#include "usv16.h"

#include <cmath>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <boost/math/constants/constants.hpp>
#include <boost/numeric/odeint.hpp>

/////////////////
// Constructor //
/////////////////

USV16::USV16(const std::vector<double>& init_velocity,
             const std::vector<double>& init_position,
             const double& main_thrust)
    : init_velocity_(init_velocity), init_position_(init_position)
{
  velocity_ << init_velocity[0], init_velocity[1], init_velocity[2];
  position_ << init_position[0], init_position[1], init_position[2];
  velocity_history_ = {velocity_};
  position_history_ = {position_};
  actuation_ << main_thrust, 0, 0;
}

/////////////////////
// Private Methods //
/////////////////////

// rotation matrix definition
Eigen::Matrix3d USV16::RotationMatrix(const Eigen::Vector3d& pose) const
{
  Eigen::Matrix3d rz;
  rz << cos(pose[2]), -sin(pose[2]), 0, sin(pose[2]), cos(pose[2]), 0, 0, 0, 1;
  return rz;
}

// inertia matrix of the vehicle
Eigen::Matrix3d USV16::InertiaMatrix() const
{
  Eigen::Matrix3d inertia_mat;
  inertia_mat << mass_ + a11_, 0, 0, 0, mass_ + a22_, a23_, 0, a23_,
      Izz_ + a33_;
  return inertia_mat;
}

// inertial centripetal and coriolis matrix
Eigen::Matrix3d USV16::InertiaCCMatrix(const Eigen::Vector3d& vec) const
{
  Eigen::Matrix3d inertia_cc_mat;
  inertia_cc_mat << 0, -mass_* vec[2], -a23_* vec[2] - a22_* vec[1],
      mass_* vec[2], 0, a11_* vec[0], a23_* vec[2] + a22_* vec[1],
      -a11_* vec[0], 0;
  return inertia_cc_mat;
}

// damping matrix of the vehicle
Eigen::Vector3d USV16::DampingVector(const Eigen::Vector3d& vec) const
{
  double U =
      std::sqrt(std::pow(vec[0], 2) + std::pow(vec[1], 2)); // vehicle speed
  double Nr =
      (U > 0.3) ? (600 * U + 200) : 400; // Nr depends on the vehicle speed
  Eigen::Vector3d damping1, damping2;
  damping1 << Xu_* vec[0], Yv_* vec[1] + Yr_* vec[2], Yr_* vec[1] + Nr* vec[2];

  // Xu|u| is different in forward and reverse conditions
  double Xuu = (vec[0] > 0) ? 112.4 : 198.1;
  damping2 << Xuu* vec[0] * std::fabs(vec[0]), 0, 0;
  return damping1 + damping2;
}

// time derivative of the system state
USV16::Vector6d USV16::StateDerivative(const Eigen::Vector3d& velocity,
                                       const Eigen::Vector3d& position,
                                       const double) const
{
  USV16::Vector6d state_derivative;
  Eigen::Vector3d vec1, vec2;

  vec1 = InertiaMatrix().inverse() * (actuation_ - DampingVector(velocity) -
                                      InertiaCCMatrix(velocity) * velocity);
  vec2 = RotationMatrix(position) * velocity;
  state_derivative << vec1, vec2;
  return state_derivative;
}

// class operator to integrate
void USV16::operator()(const std::vector<double>& state,
                       std::vector<double>& state_derivative, const double t)
{
  Eigen::Vector3d velocity, position;
  USV16::Vector6d dy;
  velocity << state[0], state[1], state[2];
  position << state[3], state[4], state[5];
  dy = StateDerivative(velocity, position, t);
  for (size_t index = 0; index < 6; ++index)
  {
    state_derivative[index] = dy[index];
  }
}

// conduct a simulation of the vehicle's motion
void RunVehicle(USV16& vehicle, const size_t& step_number = 600,
                const double& step_size = 0.1)
{
  vehicle.step_number_ = step_number;
  vehicle.step_size_ = step_size;
  // ode solver
  boost::numeric::odeint::runge_kutta_dopri5<std::vector<double>> stepper;

  std::vector<double> state;

  for (size_t index = 0; index < 3; ++index)
  {
    state.push_back(vehicle.velocity_[index]);
  }
  for (size_t index = 0; index < 3; ++index)
  {
    state.push_back(vehicle.position_[index]);
  }
  for (size_t counter = 1; counter <= step_number; ++counter)
  {
    // call the ode solver
    stepper.do_step(vehicle, state, vehicle.current_time_, step_size);

    // update the data members of the object
    ++vehicle.step_counter_;
    vehicle.current_time_ += step_size;
    vehicle.time_vec_.push_back(vehicle.current_time_);
    vehicle.velocity_ << state[0], state[1], state[2];
    vehicle.position_ << state[3], state[4], state[5];
    // wrap the heading angle
    vehicle.position_(2) = remainder(vehicle.position_[2], 2 * M_PI);
    vehicle.velocity_history_.push_back(vehicle.velocity_);
    vehicle.position_history_.push_back(vehicle.position_);
  }
}
