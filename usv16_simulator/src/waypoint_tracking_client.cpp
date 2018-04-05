#include "waypoint_tracking_client.h"

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Pose2D.h>
#include "usv16/WaypointTrackingAction.h"
#include "usv16/task_input.h"

void WaypointTrackingClient::send_goal(Maneuver* mission)
{
  // define the waypoints
  usv16::WaypointTrackingGoal goal;

  goal.mission_type = mission->get_type();
  goal.mission_duration = mission->get_duration();

  std::vector<double> xcoor = mission->get_xcoor();
  std::vector<double> ycoor = mission->get_ycoor();
  std::vector<double> heading = mission->get_heading();

  if (mission->get_progress() <
      mission->get_size()) // no longer used since the lates update
  {
    for (auto p = xcoor.begin() + mission->get_progress(); p != xcoor.end();
         ++p)
    {
      goal.pos_x.push_back(*p);
    }

    for (auto p = ycoor.begin() + mission->get_progress(); p != ycoor.end();
         ++p)
    {
      goal.pos_y.push_back(*p);
    }

    for (auto p = heading.begin() + mission->get_progress(); p != heading.end();
         ++p)
    {
      goal.heading.push_back(*p);
    }
  }
  //    else
  //    {
  //        ROS_ERROR("%s is already completed.", mission->get_name().c_str());
  //        ROS_ERROR("Reset its progress before running it again.");
  //        return;
  //    }

  if (goal.pos_x.size() != goal.pos_y.size())
  {
    ROS_ERROR_STREAM(
        "The sizes of the coordinates don't match. No waypoints sent.");
    return;
  }

  if (goal.pos_x.empty() || goal.pos_y.empty())
  {
    ROS_ERROR_STREAM("The coordinates are empty. No waypoints sent");
    return;
  }

  if (ptr_mission_ != nullptr)
  {
    ROS_WARN("------ Previous mission ended (%u/%lu) ------",
             ptr_mission_->get_progress(), ptr_mission_->get_size() - 1);
  }

  // link the pointer to the current mission
  ptr_mission_ = mission;

  // reset the status variables
  init_progress_ = ptr_mission_->get_progress();
  mission_id_ = ptr_mission_->get_name();
  is_done_ = false;

  // display the information of the mission
  ROS_INFO("----- New maneuver sent to the server -----");
  ROS_INFO("Maneuver name: %s", ptr_mission_->get_name().c_str());
  //    ROS_INFO("Initial progress: %u/%lu", init_progress_,
  //    ptr_mission_->get_size()-1);
  ROS_INFO("Number of sent waypoints: %lu", goal.pos_x.size() - 1);
  ROS_INFO("The coordinates of the waypoints are:");
  ROS_INFO("wpt ----- x (m) ------- y (m)");

  for (uint32_t i = 0; i < goal.pos_x.size(); ++i)
  {
    if (i > 0)
    {
      ROS_INFO("%2u ------ %5.2f ------- %5.2f",
               i + ptr_mission_->get_progress(), goal.pos_x[i], goal.pos_y[i]);
    }
    else
    {
      ROS_INFO("%2u ------ %5.2f ------- %5.2f", i, goal.pos_x[i],
               goal.pos_y[i]);
    }
  }

  // send the goal to the action server
  ac_.sendGoal(goal, boost::bind(&WaypointTrackingClient::doneCb, this, _1, _2),
               boost::bind(&WaypointTrackingClient::activeCb, this),
               boost::bind(&WaypointTrackingClient::feedbackCb, this, _1));
}
