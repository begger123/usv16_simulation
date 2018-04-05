#include <ros/ros.h>
#include "ship_dynamics_interface.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ship_dynamics");

  ShipDynamicsInterface usv16_interface(0.05);

  return usv16_interface.run();
}
