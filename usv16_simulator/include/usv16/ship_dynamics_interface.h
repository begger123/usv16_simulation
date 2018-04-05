#ifndef SHIP_DYNAMICS_INTERFACE_H
#define SHIP_DYNAMICS_INTERFACE_H

#include <ros/ros.h>
#include <memory>
#include <cstdlib>
#include <geometry_msgs/Twist.h>
#include "usv16.h"
#include <geometry_msgs/Pose2D.h>
#include "usv16/control.h"

class ShipDynamicsInterface
{

public:
  /////////////////
  // Constructor //
  /////////////////

  ShipDynamicsInterface(const double& timer_step);

  ///////////////////
  // Public Method //
  ///////////////////

  int run()
  {
    ros::spin();
    return EXIT_FAILURE;
  }

private:
  ////////////////////
  // Private Fields //
  ////////////////////

  // ros node
  ros::NodeHandle nh_;

  // timer which controls simulatoin and the publishing rate
  ros::Timer timer_;

  // publishers and messages
  std::unique_ptr<ros::Publisher> pubPtr_vel_;
  std::unique_ptr<ros::Publisher> pubPtr_pos_;
  geometry_msgs::Twist msg_vel_;
  geometry_msgs::Pose2D msg_pos_;

  // subscriber
  std::unique_ptr<ros::Subscriber> subPtr_act_;

  // vehicle for the simulation
  std::unique_ptr<USV16> Ptr_vehicle_;

  // time step of message publishing
  double step_size_ = 0.05;

  // flag
  bool received_control_;

  /////////////////////
  // Private Methods //
  /////////////////////

  void get_ros_param();
  void timerCallback(const ros::TimerEvent&);
  void callback_sub(const usv16::control& msg_ctrl);
};

#endif // SHIP_DYNAMICS_INTERFACE_H
