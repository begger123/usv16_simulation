#ifndef PATH_PLANNER_CLIENT_H
#define PATH_PLANNER_CLIENT_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "usv16/PathPlannerAction.h"
#include <geometry_msgs/Pose2D.h>
#include "usv16/task_input.h"
#include "maneuver.h"

class PathPlannerClient
{
  friend class UserInterface;

private:
  ////////////////////
  // Private Fields //
  ////////////////////

  // simple action client
  actionlib::SimpleActionClient<usv16::PathPlannerAction> ac_;

  // result
  usv16::PathPlannerResultConstPtr result_;

public:
  /////////////////
  // Constructor //
  /////////////////

  PathPlannerClient(const std::string& server_name) : ac_(server_name, true)
  {
    ROS_INFO("Waiting for the path planner to start.");
    ac_.waitForServer();
    ROS_INFO("Path planner started.");
  }

  ////////////////
  // Destructor //
  ////////////////

  ~PathPlannerClient() {}

  ////////////////////
  // Public Methods //
  ////////////////////

  bool compute_path(usv16::PathPlannerGoal goal)
  {
    ac_.sendGoal(goal);

    // wait for the action to return
    bool finished_before_timeout = ac_.waitForResult(ros::Duration(10.0));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = ac_.getState();
      ROS_INFO("Path planning finished: %s", state.toString().c_str());

      if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        result_ = ac_.getResult();
        return true;
      }
      else
        return false; // fail to generate a path
    }
    else
    {
      ROS_ERROR("Path planning did not finish within 10 seconds.");
      return false;
    }
  }
};

#endif // PATH_PLANNER_CLIENT_H
