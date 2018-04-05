#include "ship_dynamics_interface.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include "usv16.h"
#include "usv16/control.h"

/////////////////
// Constructor //
/////////////////

ShipDynamicsInterface::ShipDynamicsInterface(const double& timer_step)
    : step_size_(timer_step)
{
  // define the timer
  timer_ = nh_.createTimer(ros::Duration(step_size_),
                           &ShipDynamicsInterface::timerCallback, this);

  // define the publishers
  pubPtr_vel_ = std::unique_ptr<ros::Publisher>(new ros::Publisher(
      nh_.advertise<geometry_msgs::Twist>("ship/vel", 1000)));
  pubPtr_pos_ = std::unique_ptr<ros::Publisher>(new ros::Publisher(
      nh_.advertise<geometry_msgs::Pose2D>("ship/pose", 1000)));

  // define the subscriber
  subPtr_act_ =
      std::unique_ptr<ros::Subscriber>(new ros::Subscriber(nh_.subscribe(
          "ship/actuation", 1000, &ShipDynamicsInterface::callback_sub, this)));

  // get the ros parameters and define the vehicle
  get_ros_param();
}

/////////////////////
// Private Methods //
/////////////////////

void ShipDynamicsInterface::get_ros_param()
{
  // define parameters for the ship's initial position
  const std::string PARAM_1 = "/ctrl_params/initialPosition/x";
  const std::string PARAM_2 = "/ctrl_params/initialPosition/y";
  const std::string PARAM_3 = "/ctrl_params/initialPosition/psi";

  double init_x, init_y, init_heading;

  bool ok_1 = ros::param::get(PARAM_1, init_x);
  bool ok_2 = ros::param::get(PARAM_2, init_y);
  bool ok_3 = ros::param::get(PARAM_3, init_heading);

  if (ok_1 && ok_2 && ok_3)
  {
    Ptr_vehicle_ = std::unique_ptr<USV16>(
        new USV16({0, 0, 0}, {init_x, init_y, init_heading}, 0));
  }
  else
  {
    ROS_WARN("Initial position cannot be loaded. Use default values.");
    Ptr_vehicle_ = std::unique_ptr<USV16>(new USV16());
  }
}

void ShipDynamicsInterface::timerCallback(const ros::TimerEvent&)
{
  // publish the motion information of the vehicle
  ROS_INFO("Time: %5.2f s, Main thrust: %5.2f N, Yaw moment: %5.2f Nm",
           Ptr_vehicle_->current_time_, Ptr_vehicle_->actuation_[0],
           Ptr_vehicle_->actuation_[2]);
  msg_vel_.linear.x = Ptr_vehicle_->velocity_[0];
  msg_vel_.linear.y = Ptr_vehicle_->velocity_[1];
  msg_vel_.linear.z = 0;
  msg_vel_.angular.x = 0;
  msg_vel_.angular.y = 0;
  msg_vel_.angular.z = Ptr_vehicle_->velocity_[2];

  msg_pos_.x = Ptr_vehicle_->position_[0];
  msg_pos_.y = Ptr_vehicle_->position_[1];
  msg_pos_.theta = Ptr_vehicle_->position_[2];

  pubPtr_vel_->publish(msg_vel_);
  pubPtr_pos_->publish(msg_pos_);

  // run a one-step simulation
  // the time step should be equal to the publishing rate to simulate the real
  // time
  if (!received_control_)
  {
    Ptr_vehicle_->actuation_[0] = 0;
    Ptr_vehicle_->actuation_[1] = 0;
    Ptr_vehicle_->actuation_[2] = 0;
  }

  RunVehicle(*Ptr_vehicle_, 1, step_size_);

  received_control_ = false;
}

void ShipDynamicsInterface::callback_sub(const usv16::control& msg_ctrl)
{
  received_control_ = true;

  Ptr_vehicle_->actuation_[0] = msg_ctrl.surge;
  Ptr_vehicle_->actuation_[1] = msg_ctrl.sway;
  Ptr_vehicle_->actuation_[2] = msg_ctrl.yaw;
}
