#include <ros/ros.h>
#include "thrust_allocation_law.h"
#include "usv16/actuator.h"
#include "usv16/control.h"

ros::Publisher pub_actuator;
ros::Publisher pub_actuation;

// messages
usv16::actuator msg_actuator;
usv16::control msg_actuation;

// thrust allocaton law
thrust_allocaton_params ta_param = {
    120, // T_fwd
    100, // T_rev
    2,   // lx
    1    // ly
};

ThrustAllocation thrust_allocation(ta_param.T_fwd, ta_param.T_rev, ta_param.lx,
                                   ta_param.ly);

// input of thrust allocation law
std::array<double, 3> ta_input = {0, 0, 0};

// output of thrust allocation law
std::array<double, 7> output = {0, 0, 0, 0, 0, 0, 0};

void SubCallback(const usv16::controlConstPtr& actuation)
{
  // set the control signal
  ta_input[0] = actuation->surge;
  ta_input[1] = actuation->sway;
  ta_input[2] = actuation->yaw;

  // thrust allocation
  output = thrust_allocation.ComputeOutput(ta_input);

  // set the actuation signal
  msg_actuation.surge = output[4];
  msg_actuation.sway = output[5];
  msg_actuation.yaw = output[6];

  // set the actuator commmand signal
  msg_actuator.T_p = output[0];
  msg_actuator.T_s = output[2];
  msg_actuator.Alpha_p = (output[0] != 0)
                             ? atan(output[1] / output[0]) * 180 / M_PI
                             : 0; // in degree
  msg_actuator.Alpha_s = (output[2] != 0)
                             ? atan(output[3] / output[2]) * 180 / M_PI
                             : 0; // in degree

  // publish the messages
  pub_actuation.publish(msg_actuation);
  pub_actuator.publish(msg_actuator);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "thrust_allocation");
  ros::NodeHandle nh;

  pub_actuation = nh.advertise<usv16::control>("ship/actuation", 1000);
  pub_actuator = nh.advertise<usv16::actuator>("ship/actuator_cmd", 1000);
  ros::Subscriber sub_actuation =
      nh.subscribe("ship/raw_actuation", 1000, &SubCallback);

  ros::spin();

  return 0;
}
