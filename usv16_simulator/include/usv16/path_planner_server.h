#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <visualization_msgs/MarkerArray.h>
#include "usv16/PathPlannerAction.h"

class PathPlannerServer
{
protected:
  //////////////////////
  // Protected Fields //
  //////////////////////

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<usv16::PathPlannerAction> as_;

  // the publishers
  ros::Publisher pub_markerArray_;

  // the messages
  visualization_msgs::MarkerArray markerArray_obstacle_;

  // the timer which controls the publication of marker array to rviz
  ros::Timer timer_;

  // the time step of the timer
  double timer_step_;

  // the generated waypoints
  std::vector<double> waypoints_xcoor_;
  std::vector<double> waypoints_ycoor_;
  std::vector<double> waypoints_heading_;

public:
  /////////////////
  // Constructor //
  /////////////////

  PathPlannerServer(std::string);

  ////////////////
  // Destructor //
  ////////////////

  ~PathPlannerServer(void) {}

  ////////////////////
  // Public Methods //
  ////////////////////

  void executeCallback(const usv16::PathPlannerGoalConstPtr& goal);
  void timerCallback(const ros::TimerEvent& event);
};

#endif // PATH_PLANNER_H
