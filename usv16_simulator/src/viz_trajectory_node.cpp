#include <ros/ros.h>
#include "viz_trajectory.h"

int main(int argc, char** argv)
{
  // initialize ros node
  ros::init(argc, argv, "viz_trajectory");

  VizTrajectory viz_traj;

  return viz_traj.run();
}
