#ifndef WAYPOINT_TRACKING_CLIENT_H
#define WAYPOINT_TRACKING_CLIENT_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "usv16/WaypointTrackingAction.h"
#include <geometry_msgs/Pose2D.h>
#include "usv16/task_input.h"
#include "maneuver.h"

class WaypointTrackingClient
{
private:
  ////////////////////
  // Private Fields //
  ////////////////////

  // simple action client
  actionlib::SimpleActionClient<usv16::WaypointTrackingAction> ac_;

  // pointer to the current mission
  Maneuver* ptr_mission_ = nullptr;

  // initial progress of the mission
  uint32_t init_progress_;

  // the id of the current mission
  std::string mission_id_;

  // whether the current mission is done
  bool is_done_ = false;

public:
  /////////////////
  // Constructor //
  /////////////////

  WaypointTrackingClient(const std::string& server_name)
      : ac_(server_name, true)
  {
    ROS_INFO("Waiting for the waypoint-tracking server to start.");
    ac_.waitForServer();
    ROS_INFO("Waypoint-tracking server started.");
  }

  ////////////////
  // Destructor //
  ////////////////

  ~WaypointTrackingClient() {}

  ////////////////////
  // Public Methods //
  ////////////////////

  // send goal to the action server
  void send_goal(Maneuver*);

  // Called once when the goal completes
  void doneCb(const actionlib::SimpleClientGoalState& state,
              const usv16::WaypointTrackingResultConstPtr& result)
  {
    is_done_ = true;
    ROS_INFO("Maneuver finished in state [%s]", state.toString().c_str());
    ROS_INFO("The last waypoint reached: wpt%i", result->final_wpt);
    ROS_INFO("------------------------------------------");
  }

  // Called once when the goal becomes active
  void activeCb() { ROS_INFO("The goal just went active."); }

  // Called every time feedback is received for the goal
  void feedbackCb(const usv16::WaypointTrackingFeedbackConstPtr& feedback)
  {
    ptr_mission_->set_progress(feedback->progress + init_progress_);
    ROS_INFO("Progress of the maneuver: %u/%lu", ptr_mission_->get_progress(),
             ptr_mission_->get_size() - 1);

    if (ptr_mission_->get_type() &&
        (ptr_mission_->get_progress() == ptr_mission_->get_size() - 1))
      ROS_INFO("The vehicle is now in station-keeping mode.");
  }

  // return whether the current mission is finished or not
  bool is_done() const { return is_done_; }

  // return the id of the current mission
  std::string mission_id() const { return mission_id_; }
};

#endif // WAYPOINT_TRACKING_CLIENT_H
