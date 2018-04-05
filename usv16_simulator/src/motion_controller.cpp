#include "motion_controller.h"

#include <ros/ros.h>
#include <memory>
#include <array>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <eigen3/Eigen/Dense>
#include <boost/math/special_functions/sign.hpp>
#include "pf_controller.h"
#include "sk_controller.h"
#include "thrust_allocation_law.h"
#include "usv16/course.h"
#include "usv16/control.h"

using namespace std;

/////////////////
// Constructor //
/////////////////

MotionController::MotionController(const double& u_d,
                                   const path_following::vehicle_params& pf_v,
                                   const path_following::control_params& pf_c,
                                   const station_keeping::vehicle_params& sk_v,
                                   const station_keeping::control_params& sk_c,
                                   const double& timer_step)
    : u_d_(u_d), step_size_(timer_step)
{
  // define the controllers
  pf_controller_ = path_following::PFController(pf_v, pf_c);
  sk_controller_ = station_keeping::SKController(sk_v, sk_c);

  // define the timer
  timer_ = nh_.createTimer(ros::Duration(step_size_),
                           &MotionController::timerCallback, this);

  // define the publisher
  pubPtr_control_ = unique_ptr<ros::Publisher>(new ros::Publisher(
      nh_.advertise<usv16::control>("ship/auto_control", 1000)));

  // define the subscribers
  sub_pos_ = unique_ptr<ros::Subscriber>(new ros::Subscriber(nh_.subscribe(
      "ship/pose", 1000, &MotionController::callback_pose, this)));
  sub_vel_ = unique_ptr<ros::Subscriber>(new ros::Subscriber(
      nh_.subscribe("ship/vel", 1000, &MotionController::callback_vel, this)));
  sub_course_ = unique_ptr<ros::Subscriber>(new ros::Subscriber(nh_.subscribe(
      "ship/course", 1000, &MotionController::callback_course, this)));
  sub_station_ = unique_ptr<ros::Subscriber>(new ros::Subscriber(nh_.subscribe(
      "ship/station", 1000, &MotionController::callback_station, this)));
}

////////////////////
// Private Method //
////////////////////

void MotionController::timerCallback(const ros::TimerEvent&)
{
  if (received_course_info_ && !received_station_info_)
  {
    Eigen::Vector3d vel(u_, v_, r_);
    Eigen::Vector3d pos(0, 0, psi_);

    // compute the actuation
    actuation_ = pf_controller_.ComputeActuation(vel, pos, u_d_, psi_d_);

    // set the control signal
    actuation_[0] = (actuation_[0] >= 50)
                        ? actuation_[0]
                        : 50; // non-negative in path-following mode

    // reset the flag
    received_course_info_ = false;
  }
  else if (!received_course_info_ && received_station_info_)
  {
    Eigen::Vector3d vel(u_, v_, r_);
    Eigen::Vector3d pos(x_, y_, psi_);

    // check the psi difference, find a shorter rotating direction
    double psi_d_temp =
        (fabs(psi_ - psi_d_) > M_PI)
            ? (psi_d_ + 2 * M_PI * boost::math::sign(psi_ - psi_d_))
            : psi_d_;

    Eigen::Vector3d pos_ref(x_d_, y_d_, psi_d_temp);
    Eigen::Vector3d pos_ref_dot(0, 0, 0);
    Eigen::Vector3d pos_ref_ddot(0, 0, 0);

    // calculate the actuation
    actuation_ = sk_controller_.ComputeActuation(pos, vel, pos_ref, pos_ref_dot,
                                                 pos_ref_ddot);

    // reset the flag
    received_station_info_ = false;
  }
  else if (received_course_info_ && received_station_info_)
  {
    // reset when this is a conflict
    received_course_info_ = false;
    received_station_info_ = false;
  }
  else
  {
    actuation_ = {0, 0, 0};
  }

  // set the actuation signal
  msg_actuation_.surge = actuation_[0];
  msg_actuation_.sway = actuation_[1];
  msg_actuation_.yaw = actuation_[2];

  // publish the messages
  pubPtr_control_->publish(msg_actuation_);
}
