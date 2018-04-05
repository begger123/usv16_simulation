#include <ros/ros.h>
#include "waypoint_tracking_server.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "guidance_node");

  WaypointTrackingServer waypoint_tracking(ros::this_node::getName());

  ros::spin();

  return 0;
}
