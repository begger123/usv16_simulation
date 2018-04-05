#include <ros/ros.h>
#include "path_planner_server.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_planner");

  PathPlannerServer path_planner(ros::this_node::getName());

  ros::spin();

  return 0;
}
