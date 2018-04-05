#ifndef TASK_USER_INTERFACE_H
#define TASK_USER_INTERFACE_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "maneuver_manager.h"
#include "waypoint_tracking_client.h"
#include "path_planner_client.h"
#include "usv16/WaypointTrackingAction.h"
#include <geometry_msgs/Pose2D.h>
#include "usv16/task_input.h"
#include "usv16/maneuver_list.h"

/////////////////////////
// Forward Declaration //
/////////////////////////

class Maneuver;

class UserInterface
{
private:
  ////////////////////
  // Private Fields //
  ////////////////////

  ros::NodeHandle nh_;
  ros::ServiceServer srv_user_;

  // store the real-time position of the ship
  std::array<double, 3> ship_pos_;

  // gps availability
  bool pose_available_ = false;

  // timer which sets pose_available_ and publishs trajectories of incoming
  // missions to rviz
  ros::Timer timer_;

  // publisher
  ros::Publisher pub_path_;
  ros::Publisher pub_target_;
  ros::Publisher pub_maneuvers_;

  // subscribers
  ros::Subscriber sub_pos_;
  ros::Subscriber sub_nav_goal_;

  // rviz visualization msg
  visualization_msgs::Marker marker_path_;
  visualization_msgs::Marker marker_target_;
  visualization_msgs::MarkerArray marker_target_array_;

  // time step of the timers
  double timer_step_;

  // helper variable
  bool received_command_ = false;
  bool maneuvers_changed_ = false;

  // command inputs from the user
  std::string command_name_;
  double goal_xcoor_;
  double goal_ycoor_;
  double goal_psi_;
  bool goal_station_keeping_;
  double goal_duration_;

  // command set
  const std::vector<std::string> command_set_ = {"overwrite", "append",
                                                 "insert"};

  // actionlib clients
  WaypointTrackingClient waypoint_tracking_client_{"guidance_node"};
  PathPlannerClient path_planner_client_{"planner_node"};

  // mission manager
  ManeuverManager mission_manager_;

  ////////////////////
  // Private Method //
  ////////////////////

  // callback of the timer for checking gps availability and update trajectories
  // in rviz
  void timerCallback(const ros::TimerEvent& event)
  {
    pose_available_ = false;

    // publish path to rviz
    geometry_msgs::Point p;

    // publish maneuver list
    usv16::maneuver_list maneuvers;

    marker_path_.points.clear();
    marker_target_array_.markers.clear();

    if (!mission_manager_.mission_queue_.empty())
    {
      for (auto iter = mission_manager_.mission_queue_.cbegin();
           iter != mission_manager_.mission_queue_.cend(); ++iter)
      {
        for (size_t i = 0;
             i != (mission_manager_.mission_database_.at(*iter)).get_size();
             ++i)
        {
          p.x = (mission_manager_.mission_database_.at(*iter)).wpt_xcoor_[i];
          p.y = -(mission_manager_.mission_database_.at(*iter)).wpt_ycoor_[i];
          p.z = 0;

          if (i ==
              (mission_manager_.mission_database_.at(*iter)).get_size() - 1)
          {
            marker_target_.id += 1;
            marker_target_.pose.position.x = p.x;
            marker_target_.pose.position.y = p.y;
            marker_target_.pose.position.z = p.z;

            marker_target_array_.markers.push_back(marker_target_);
          }
          marker_path_.points.push_back(p);
        }

        if (maneuvers_changed_)
          maneuvers.maneuver.push_back(
              mission_manager_.mission_database_.at(*iter).description_);
      }
      // publish the messages
      pub_path_.publish(marker_path_);
      pub_target_.publish(marker_target_array_);

      if (maneuvers_changed_)
        pub_maneuvers_.publish(maneuvers);

      // reset the flag
      maneuvers_changed_ = false;
    }
  }

public:
  /////////////////
  // Constructor //
  /////////////////

  UserInterface()
  {
    // define the subscriber to ship/pos
    sub_pos_ =
        nh_.subscribe("ship/pose", 1000, &UserInterface::callback_pos, this);

    // define the subscriber to move_base_simple/goal
    sub_nav_goal_ = nh_.subscribe("move_base_simple/goal", 10,
                                  &UserInterface::callback_nav, this);

    // define the service responding to user inputs
    srv_user_ = nh_.advertiseService("goal_assignment",
                                     &UserInterface::UserInputCallback, this);

    // define the timer used to reset gps availability
    timer_step_ = 1;
    timer_ = nh_.createTimer(ros::Duration(timer_step_),
                             &UserInterface::timerCallback, this);

    // define the publisher
    pub_path_ =
        nh_.advertise<visualization_msgs::Marker>("planner/path_marker", 1000);
    pub_target_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "manager/target_marker", 10);
    pub_maneuvers_ =
        nh_.advertise<usv16::maneuver_list>("ship/maneuvers", 1000);

    // initialize the path marker
    marker_path_.header.frame_id = "world";
    marker_path_.header.stamp = ros::Time::now();
    marker_path_.ns = "path/viz";
    marker_path_.action = visualization_msgs::Marker::ADD;
    marker_path_.pose.orientation.w = 1.0;
    marker_path_.id = 0;
    marker_path_.type = visualization_msgs::Marker::LINE_STRIP;
    marker_path_.scale.x = 0.05; // line width
    marker_path_.color.r = 0.0;
    marker_path_.color.g = 1.0;
    marker_path_.color.b = 0.0;
    marker_path_.color.a = 1.0;

    // initialize the target marker
    marker_target_.header.frame_id = "world";
    marker_target_.header.stamp = ros::Time::now();
    marker_target_.ns = "target/viz";
    marker_target_.action = visualization_msgs::Marker::ADD;
    marker_target_.pose.orientation.w = 1.0;
    marker_target_.id = 0;
    marker_target_.lifetime = ros::Duration(timer_step_);
    marker_target_.type = visualization_msgs::Marker::CYLINDER;
    marker_target_.scale.x = 5;
    marker_target_.scale.y = 5;
    marker_target_.scale.z = 1;
    marker_target_.color.r = 0.0;
    marker_target_.color.g = 0.0;
    marker_target_.color.b = 1.0;
    marker_target_.color.a = 0.3;
  }

  ////////////////////
  // Public Methods //
  ////////////////////

  // return received_command_
  bool new_input_available() { return received_command_; }

  // return pose_available
  bool gps_available() const { return pose_available_; }

  // callback function of the subscriber to ship/pose
  void callback_pos(const geometry_msgs::Pose2DConstPtr& msg_pos)
  {
    pose_available_ = true;

    ship_pos_[0] = msg_pos->x;
    ship_pos_[1] = msg_pos->y;
    ship_pos_[2] = msg_pos->theta;
  }

  // convert quaternion to Euler angles
  // note that the function is only used for 2D conversion
  void toEulerianAngle(const double& z, const double& w, double& yaw)
  {
    double t0 = -2.0f * (z * z) + 1.0f;
    double t1 = +2.0f * (w * z);
    yaw = -std::atan2(t1, t0); // the minus sign is for transformation from
                               // world frame to earth-fixed frame
  }

  // callback function of the subscriber to move_base_simple/goal
  void callback_nav(const geometry_msgs::PoseStampedConstPtr& msg_nav);

  // callback of the service for user inputs
  bool UserInputCallback(usv16::task_input::Request& req,
                         usv16::task_input::Response& resp);

  // receive user command and manage waypoint-tracking mission
  void CommandCallback();

  // generate a simple station-keeping mission to rotate the vehicle
  Maneuver GenerateSimpleSKMission(const Maneuver& mission,
                                   const double& goal_duration);

  // generate new mission based on the target position
  Maneuver GenerateMission(const double& start_xcoor, const double& start_ycoor,
                           const double& start_psi, const double& goal_xcoor,
                           const double& goal_ycoor, const double& goal_psi,
                           const bool& goal_station_keeping);

  // run
  int run();
};

#endif // TASK_USER_INTERFACE_H
