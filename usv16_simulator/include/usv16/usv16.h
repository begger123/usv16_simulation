#ifndef USV16_H
#define USV16_H

#include <cmath>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <boost/math/constants/constants.hpp>
#include <boost/numeric/odeint.hpp>

/////////////////////////
// Forward Declaration //
////////////////////////

class USV16;
void RunVehicle(USV16&, const size_t&, const double&);

// Class USV16
class USV16
{
  friend class SKController;
  friend void RunVehicle(USV16&, const size_t&, const double&);

  // constant pi
  const double pi = boost::math::constants::pi<double>();

public:
  /////////////////
  // Constructor //
  /////////////////

  USV16(const std::vector<double>& init_velocity,
        const std::vector<double>& init_position, const double& main_thrust);
  USV16() : USV16({1.0, 0, 0}, {0, 0, 0}, 0.0) {}
  USV16(const std::vector<double>& init_velocity)
      : USV16(init_velocity, {0, 0, 0}, 0.0)
  {
  }
  USV16(const std::vector<double>& init_velocity,
        const std::vector<double>& init_position)
      : USV16(init_velocity, init_position, 0.0)
  {
  }

private:
  ////////////////////
  // Private Fields //
  ////////////////////

  // new matrix types
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;

  // parameters of the vehicle
  const double gravity_ = 9.81;
  const double mass_ = 180;
  const double Izz_ = 250;
  const double a11_ = 27;
  const double a22_ = 37;
  const double a33_ = 93.2345;
  const double a23_ = 50;
  const double Xu_ = 0;
  const double Yv_ = 300;
  const double Yr_ = -40;

  // parameters of the simulation
  // step size of the current simulation
  double step_size_ = 0.0;

  // step number of the current simulation
  size_t step_number_ = 0;

  // counter of steps
  size_t step_counter_ = 0;

  // time vector of the simulation
  std::vector<double> time_vec_ = {0};

  // array of velocity and position vectors during the simulation
  std::vector<Eigen::Vector3d> velocity_history_, position_history_;

  /////////////////////
  // Private Methods //
  /////////////////////

  // rotation matrix declaration
  inline Eigen::Matrix3d RotationMatrix(const Eigen::Vector3d& pose) const;

  // inertia matrix of the vehicle
  inline Eigen::Matrix3d InertiaMatrix() const;

  // inertia centripetal and coriolis matrix of the vehicle
  inline Eigen::Matrix3d InertiaCCMatrix(const Eigen::Vector3d&) const;

  // damping vector of the vehicle
  inline Eigen::Vector3d DampingVector(const Eigen::Vector3d&) const;

  // time derivative of the system state
  inline Vector6d StateDerivative(const Eigen::Vector3d& velocity,
                                  const Eigen::Vector3d& position,
                                  const double /* t */) const;

public:
  ///////////////////
  // Public Fields //
  ///////////////////

  // elapsed time since current simulation starts
  double current_time_ = 0.0;

  // initial velocity and position(and orientation) of the vehicle
  std::vector<double> init_velocity_, init_position_;

  // current velocity and position of the vehicle
  Eigen::Vector3d velocity_, position_;

  // actuation acting on the vehicle
  Eigen::Vector3d actuation_;

  // operator function to integrate
  inline void operator()(const std::vector<double>& state,
                         std::vector<double>& state_derivative, const double);
};

#endif // USV16_H
