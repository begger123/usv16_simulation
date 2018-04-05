#include "user_interface.h"

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cmath>
#include "usv16/WaypointTrackingAction.h"
#include "usv16/PathPlannerAction.h"
#include <geometry_msgs/Pose2D.h>
#include "usv16/task_input.h"
#include "waypoint_tracking_client.h"
#include "path_planner_client.h"
#include "maneuver_manager.h"

// callback of the service for user inputs
bool UserInterface::UserInputCallback(usv16::task_input::Request& req,
                                      usv16::task_input::Response& resp)
{
  ROS_WARN("Received command from the user.");
  resp.feedback = "User\'s command is received.";

  if (std::find(command_set_.begin(), command_set_.end(), req.command) ==
      command_set_.end())
  {
    ROS_ERROR("Your command is not valid.");
    resp.feedback += " Your command is not valid.";
  }
  else
  {
    command_name_ = req.command;
    goal_xcoor_ = req.target_x;
    goal_ycoor_ = req.target_y;
    goal_psi_ = req.target_psi * M_PI / 180; // in radian
    goal_station_keeping_ = req.station_keeping;
    goal_duration_ = req.duration;

    // set the flag
    received_command_ = true;
  }

  return true;
}

// callback of the subscriber to move_base_simple/goal
void UserInterface::callback_nav(
    const geometry_msgs::PoseStampedConstPtr& msg_nav)
{
  ROS_WARN("Received a navigation goal from rviz.");
  toEulerianAngle(msg_nav->pose.orientation.z, msg_nav->pose.orientation.w,
                  goal_psi_);
  goal_xcoor_ = msg_nav->pose.position.x;
  goal_ycoor_ = -msg_nav->pose.position.y; // world frame to earth frame
  command_name_ = "overwrite";             // overwrite the current mission
  goal_station_keeping_ = false;
  goal_duration_ = 0;

  // set the flag
  received_command_ = true;
}

// generate new mission based on the target position
Maneuver UserInterface::GenerateMission(
    const double& start_xcoor, const double& start_ycoor,
    const double& start_psi, const double& goal_xcoor, const double& goal_ycoor,
    const double& goal_psi, const bool& station_keeping)
{
  // define the goal for the path planner
  usv16::PathPlannerGoal goal;

  // check the bound of the goal for the path planner
  double start_psi_temp = (start_psi < 0) ? (start_psi + 2 * M_PI) : start_psi;
  double end_psi_temp = (goal_psi < 0) ? (goal_psi + 2 * M_PI) : goal_psi;

  // prepare the goal for the path planner
  goal.start_pose = {start_xcoor, start_ycoor, start_psi_temp};
  goal.start_vel = {1.5, 0};
  goal.end_pose = {goal_xcoor, goal_ycoor, end_psi_temp};

  // compute a feasible path
  bool success = path_planner_client_.compute_path(goal);

  Maneuver mission;

  if (success)
  {
    mission.goal_xcoor_ = goal_xcoor;
    mission.goal_ycoor_ = goal_ycoor;
    mission.goal_psi_ = goal_psi;
    mission.wpt_xcoor_ = path_planner_client_.result_->wpt_x;
    mission.wpt_ycoor_ = path_planner_client_.result_->wpt_y;
    mission.wpt_heading_ = path_planner_client_.result_->wpt_psi;
    mission.wpt_xcoor_.pop_back();
    mission.wpt_ycoor_.pop_back();
    mission.wpt_heading_.pop_back();
    mission.wpt_xcoor_.push_back(goal_xcoor);
    mission.wpt_ycoor_.push_back(goal_ycoor);
    mission.wpt_heading_.push_back(goal_psi);

    mission.duration_ = ros::Duration(goal_duration_);
    mission.id_ = std::to_string(ros::Time::now().toSec());
    mission.station_keeping_on_ = station_keeping;

    if (station_keeping)
    {
      mission.description_ = "Transit to and keep position at (" +
                             std::to_string(int(goal_xcoor)) + ", " +
                             std::to_string(int(goal_ycoor)) + ", " +
                             std::to_string(int(goal_psi)) + ")";
    }
    else
    {
      mission.description_ = "Transit to  (" + std::to_string(int(goal_xcoor)) +
                             ", " + std::to_string(int(goal_ycoor)) + ")";
    }

    // update the mission manager
    mission_manager_.mission_database_[mission.get_name()] = mission;
    mission_manager_.mission_set_.insert(mission.get_name());
  }

  return mission;
}

// insert a simple station-keeping mission to rotate the vehicle
Maneuver UserInterface::GenerateSimpleSKMission(const Maneuver& next_mission,
                                                const double& goal_duration)
{
  Maneuver mission;

  mission.wpt_xcoor_.push_back(next_mission.wpt_xcoor_[0]);
  mission.wpt_ycoor_.push_back(next_mission.wpt_ycoor_[0]);
  mission.wpt_heading_.push_back(
      std::atan2(next_mission.wpt_ycoor_[1] - next_mission.wpt_ycoor_[0],
                 next_mission.wpt_xcoor_[1] - next_mission.wpt_xcoor_[0]));

  mission.duration_ = ros::Duration(goal_duration);
  mission.id_ = std::to_string((ros::Time::now().toSec()));
  mission.station_keeping_on_ = true;
  mission.description_ =
      "Rotate towards " +
      std::to_string(int(180 * mission.wpt_heading_.at(0) / M_PI)) + " deg";

  // update the mission manager
  mission_manager_.mission_database_[mission.get_name()] = mission;
  mission_manager_.mission_set_.insert(mission.get_name());

  return mission;
}

// receive user command and manage missions
void UserInterface::CommandCallback()
{
  if (mission_manager_.mission_queue_.empty()) // mission queue is empty
  {
    Maneuver mission =
        GenerateMission(ship_pos_[0], ship_pos_[1], ship_pos_[2], goal_xcoor_,
                        goal_ycoor_, goal_psi_, goal_station_keeping_);

    if (!mission.empty()) // valid mission
    {
      Maneuver sk_mission = GenerateSimpleSKMission(
          mission,
          10); // a simple station-keeping mission to rotate the vehicle
      mission_manager_.mission_queue_.push_back(
          sk_mission.get_name()); // push back the simple sk mission
      mission_manager_.mission_queue_.push_back(
          mission.get_name()); // push back the normal mission again
      waypoint_tracking_client_.send_goal(
          &(mission_manager_.mission_database_.at(
              mission_manager_.mission_queue_.front())));
      ROS_INFO("New maneuver is generated, and sent to the waypoint-tracking "
               "server.");
    }
    else
      ROS_ERROR("A feasible path cannot be found!!!");
  }
  else // mission queue is not empty
  {
    if (command_name_ == "append")
    {
      Maneuver last_mission = mission_manager_.mission_database_.at(
          mission_manager_.mission_queue_.back());
      Maneuver mission =
          GenerateMission(last_mission.goal_xcoor_, last_mission.goal_ycoor_,
                          last_mission.goal_psi_, goal_xcoor_, goal_ycoor_,
                          goal_psi_, goal_station_keeping_);

      if (!mission.empty()) // valid mission
      {
        Maneuver sk_mission = GenerateSimpleSKMission(
            mission,
            10); // a simple station-keeping mission to rotate the vehicle
        mission_manager_.mission_queue_.push_back(
            sk_mission.get_name()); // push back the simple sk mission
        mission_manager_.mission_queue_.push_back(
            mission.get_name()); // push back the normal mission again
        ROS_INFO("New maneuvers are generated, and appended to the queue.");
      }
      else
        ROS_ERROR("A feasible path cannot be found!!!");
    }

    if (command_name_ == "overwrite")
    {
      Maneuver* current_mission = &(mission_manager_.mission_database_.at(
          mission_manager_.mission_queue_.front()));

      size_t index =
          ((current_mission->get_progress() == current_mission->get_size() - 1)
               ? index = current_mission->get_progress()
               : current_mission->get_progress() + 1);

      Maneuver mission = GenerateMission(current_mission->wpt_xcoor_[index],
                                         current_mission->wpt_ycoor_[index],
                                         ship_pos_[2], goal_xcoor_, goal_ycoor_,
                                         goal_psi_, goal_station_keeping_);

      if (!mission.empty()) // valid mission
      {
        mission_manager_.mission_queue_.front() = mission.get_name();
        waypoint_tracking_client_.send_goal(
            &(mission_manager_.mission_database_.at(
                mission_manager_.mission_queue_.front())));
        ROS_INFO(
            "New mission is generated, and overrides the current mission.");

        // the current target has changed, therefore, we need to
        // change the mission (including the simple sk mission) following the
        // current mission, if there is any.
        if (mission_manager_.mission_queue_.size() >= 3)
        {
          Maneuver mission = GenerateMission(
              (mission_manager_.mission_database_.at(
                   mission_manager_.mission_queue_.front())).goal_xcoor_,
              (mission_manager_.mission_database_.at(
                   mission_manager_.mission_queue_.front())).goal_ycoor_,
              (mission_manager_.mission_database_.at(
                   mission_manager_.mission_queue_.front())).goal_psi_,
              (mission_manager_.mission_database_.at(
                   mission_manager_.mission_queue_[2])).goal_xcoor_,
              (mission_manager_.mission_database_.at(
                   mission_manager_.mission_queue_[2])).goal_ycoor_,
              (mission_manager_.mission_database_.at(
                   mission_manager_.mission_queue_[2])).goal_psi_,
              (mission_manager_.mission_database_.at(
                   mission_manager_.mission_queue_[2])).station_keeping_on_);

          if (!mission.empty()) // valid mission
          {
            // a simple station-keeping mission to rotate the vehicle
            Maneuver sk_mission = GenerateSimpleSKMission(mission, 10);

            // replace the missions with new ones
            mission_manager_.mission_queue_[1] =
                sk_mission.get_name(); // replace the simple sk mission
            mission_manager_.mission_queue_[2] =
                mission.get_name(); // replace the normal mission
            ROS_INFO("The succeeding mission is also updated.");
          }
          else
          {
            ROS_ERROR("The succeeding mission cannot be updated, not path can "
                      "be found!!!");
            ROS_ERROR("All the succeeding missions are aborted!!!");
            mission_manager_.mission_queue_.erase(
                mission_manager_.mission_queue_.begin() + 1,
                mission_manager_.mission_queue_.end());
          }
        }
        else if (mission_manager_.mission_queue_.size() == 2)
        {
          mission_manager_.mission_queue_.pop_back();
        }
      }
      else
        ROS_ERROR("A feasible path cannot be found!!!");
    }

    if (command_name_ == "insert")
    {
      Maneuver* current_mission = &(mission_manager_.mission_database_.at(
          mission_manager_.mission_queue_.front()));

      size_t index =
          ((current_mission->get_progress() == current_mission->get_size() - 1)
               ? index = current_mission->get_progress()
               : current_mission->get_progress() + 1);

      Maneuver mission = GenerateMission(current_mission->wpt_xcoor_[index],
                                         current_mission->wpt_ycoor_[index],
                                         ship_pos_[2], goal_xcoor_, goal_ycoor_,
                                         goal_psi_, goal_station_keeping_);

      if (!mission.empty()) // valid mission
      {
        mission_manager_.mission_queue_.push_front(mission.get_name());
        waypoint_tracking_client_.send_goal(
            &(mission_manager_.mission_database_.at(
                mission_manager_.mission_queue_.front())));
        ROS_INFO(
            "New mission is generated, and overrides the current mission.");

        // new target inserted, therefore, we need to change the following
        // mission
        Maneuver next_mission = GenerateMission(
            (mission_manager_.mission_database_.at(
                 mission_manager_.mission_queue_.front())).goal_xcoor_,
            (mission_manager_.mission_database_.at(
                 mission_manager_.mission_queue_.front())).goal_ycoor_,
            (mission_manager_.mission_database_.at(
                 mission_manager_.mission_queue_.front())).goal_psi_,
            (mission_manager_.mission_database_.at(
                 mission_manager_.mission_queue_[1])).goal_xcoor_,
            (mission_manager_.mission_database_.at(
                 mission_manager_.mission_queue_[1])).goal_ycoor_,
            (mission_manager_.mission_database_.at(
                 mission_manager_.mission_queue_[1])).goal_psi_,
            (mission_manager_.mission_database_.at(
                 mission_manager_.mission_queue_[1])).station_keeping_on_);

        if (!next_mission.empty())
        {
          // a simple station-keeping mission to rotate the vehicle
          Maneuver sk_mission = GenerateSimpleSKMission(next_mission, 10);

          // replace the missions with new ones
          mission_manager_.mission_queue_[1] = next_mission.get_name();
          mission_manager_.mission_queue_.insert(
              mission_manager_.mission_queue_.begin() + 1,
              sk_mission.get_name());
          ROS_INFO("The succeeding mission is also updated.");
        }
        else
        {
          ROS_ERROR("The succeeding mission cannot be updated, no path can be "
                    "found!!!");
          ROS_ERROR("All the succeeding missions are aborted!!!");
          mission_manager_.mission_queue_.erase(
              mission_manager_.mission_queue_.begin() + 1,
              mission_manager_.mission_queue_.end());
        }
      }
      else
        ROS_ERROR("A feasible path cannot be found!!!");
    }
  }

  // reset the flag
  received_command_ = false;
  maneuvers_changed_ = true;
}

// run
int UserInterface::run()
{
  ros::Rate r(5);

  while (ros::ok())
  {
    if (this->gps_available())
    {
      // keep checking the incoming user command
      if (this->new_input_available())
      {
        this->CommandCallback();
      }

      // keep checking whether the current mission is done
      if (waypoint_tracking_client_.is_done())
      {
        if (!mission_manager_.mission_queue_.empty())
        {
          mission_manager_.mission_queue_.pop_front();
          if (!mission_manager_.mission_queue_.empty())
          {
            ROS_WARN("Send the next mission in the queue.");
            waypoint_tracking_client_.send_goal(
                &(mission_manager_.mission_database_.at(
                    mission_manager_.mission_queue_.front())));
          }
          maneuvers_changed_ = true;
        }
        else
        {
          ROS_WARN_THROTTLE(10, "No more missions in the queue.");
        }
      }
    }

    ros::spinOnce();
    r.sleep();
  }

  return EXIT_FAILURE;
}
