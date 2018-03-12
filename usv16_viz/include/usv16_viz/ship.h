#ifndef SHIP_H
#define SHIP_H

#include <cmath>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <boost/math/constants/constants.hpp>
#include <boost/numeric/odeint.hpp>

// constant pi
const double pi = boost::math::constants::pi<double>();
// new matrix types
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

// Class Remus
class Ship
{
    friend class HeadingController;
    friend void RunShip(Ship&, const size_t&, const double&);

public:
    // constructors
    Ship(const std::vector<double> &init_velocity, const std::vector<double> &init_position, const double &main_thrust);
    Ship(): Ship({0.1, 0, 0}, {0, 0, 0}, 0.2) {}
    Ship(const std::vector<double> &init_velocity): Ship(init_velocity, {0, 0, 0}, 0.2) {}
    Ship(const std::vector<double> &init_velocity, const std::vector<double> &init_position):
            Ship(init_velocity, init_position, 0.2) {}

private:
    // parameters of the vehicle
    const double gravity_ = 9.81;
    const double m11_ = 25.8;
    const double m22_ = 33.8;
    const double m23_ = 1.0115;
    const double m33_ = 2.76;
    const double n11_ = 2.0;
    const double n22_ = 7.0;
    const double n23_ = 0.1;
    const double n33_ = 0.5;
    const double arm_ = 0.6; // N_{delta}/Y_{delta}

    // parameters of the simulation
    // step size of the current simulation
    double step_size_ = 0.0;
    // step number of the current simulation
    size_t step_number_ = 0;
    // counter of steps
    size_t step_counter_ = 0;

public:
    // elapsed time since current simulation starts
    double current_time_ = 0.0;
    // initial velocity and position(and orientation) of the vehicle
    std::vector<double> init_velocity_, init_position_;
    // current velocity and position of the vehicle
    Eigen::Vector3d velocity_, position_;
    // time vector of the simulation
    std::vector<double> time_vec_ = {0};
    // array of velocity and position vectors during the simulation
    std::vector<Eigen::Vector3d> velocity_history_, position_history_;
    // actuation acting on the vehicle
    Eigen::Vector3d actuation_;

    // rotation matrix declaration
    static Eigen::Matrix3d RotationMatrix(const Eigen::Vector3d &pose);

    // inertia matrix of the ship
    Eigen::Matrix3d InertiaMatrix() const;

    // damping matrix of the ship
    Eigen::Matrix3d DampingMatrix() const;

    // time derivative of the system state
    Vector6d StateDerivative(const Eigen::Vector3d &velocity, const Eigen::Vector3d &position, const double /* t */) const;
    // operator function to integrate
    void operator() (const std::vector<double> &state, std::vector<double> &state_derivative, const double);
};

// constructor definition
Ship::Ship(const std::vector<double> &init_velocity, const std::vector<double> &init_position, const double &main_thrust):
        init_velocity_(init_velocity), init_position_(init_position) {
    velocity_ << init_velocity[0], init_velocity[1], init_velocity[2];
    position_ << init_position[0], init_position[1], init_position[2];
    velocity_history_ = {velocity_};
    position_history_ = {position_};
    actuation_ << main_thrust, 0, 0;
}

// rotation matrix definition
Eigen::Matrix3d Ship::RotationMatrix(const Eigen::Vector3d &pose) {
    Eigen::Matrix3d rz;
    rz << cos(pose[2]), -sin(pose[2]), 0,
          sin(pose[2]), cos(pose[2]), 0,
          0, 0, 1;
    return rz;
}

// inertia matrix of the ship
Eigen::Matrix3d Ship::InertiaMatrix() const {
    Eigen::Matrix3d inertia_mat;
    inertia_mat << m11_, 0, 0,
                   0, m22_, m23_,
                   0, m23_, m33_;
    return inertia_mat;
}

// damping matrix of the ship
Eigen::Matrix3d Ship::DampingMatrix() const {
    Eigen::Matrix3d damping_mat;
    damping_mat << n11_, 0, 0,
                   0, n22_, n23_,
                   0, n23_, n33_;
    return damping_mat;
}

// time derivative of the system state
Vector6d Ship::StateDerivative(const Eigen::Vector3d &velocity, const Eigen::Vector3d &position, const double) const {
    Vector6d state_derivative;
    Eigen::Vector3d vec1, vec2;

    vec1 = InertiaMatrix().inverse() *
           (actuation_ - DampingMatrix() * velocity);
    vec2 = RotationMatrix(position) * velocity;
    state_derivative << vec1, vec2;
    return state_derivative;
}

// class operator to integrate
void Ship::operator()(const std::vector<double> &state, std::vector<double> &state_derivative, const double t) {
    Eigen::Vector3d velocity, position;
    Vector6d dy;
    velocity << state[0], state[1], state[2];
    position << state[3], state[4], state[5];
    dy = StateDerivative(velocity, position, t);
    for (size_t index = 0; index < 6; ++index) {
        state_derivative[index] = dy[index];
    }
}

// conduct a simulation of the vehicle's motion
void RunShip(Ship &vehicle, const size_t &step_number = 600, const double &step_size = 0.1) {
    vehicle.step_number_ = step_number;
    vehicle.step_size_ = step_size;
    // ode solver
    boost::numeric::odeint::runge_kutta_dopri5 <std::vector<double>> stepper;

    std::vector<double> state;

    for (size_t index = 0; index < 3; ++index) {
        state.push_back(vehicle.velocity_[index]);
    }
    for (size_t index = 0; index < 3; ++index) {
        state.push_back(vehicle.position_[index]);
    }
    for (size_t counter = 1; counter <= step_number; ++counter) {
        // call the ode solver
        stepper.do_step(vehicle, state, vehicle.current_time_, step_size);

        // update the data members of the object
        ++vehicle.step_counter_;
        vehicle.current_time_ += step_size;
        vehicle.time_vec_.push_back(vehicle.current_time_);
        vehicle.velocity_ << state[0], state[1], state[2];
        vehicle.position_ << state[3], state[4], state[5];
        // wrap the heading angle
        vehicle.position_(2) = remainder(vehicle.position_[2], 2*M_PI);
        vehicle.velocity_history_.push_back(vehicle.velocity_);
        vehicle.position_history_.push_back(vehicle.position_);
    }
}

#endif // SHIP_H
