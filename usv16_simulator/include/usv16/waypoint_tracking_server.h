#ifndef WAYPOINT_TRACKING_SERVER_H
#define WAYPOINT_TRACKING_SERVER_H

#include <vector>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose2D.h>
#include "usv16/course.h"
#include "usv16/waypoint.h"
#include "los_guidance_law.h"
#include "usv16/WaypointTrackingAction.h"

class WaypointTrackingServer
{
protected:
  //////////////////////
  // Protected Fields //
  //////////////////////

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<usv16::WaypointTrackingAction> as_;

  // the timer which controls the publication of markers to rviz
  ros::Timer timer_;

  // the time step of the timer
  double timer_step_;

  // the publishers
  ros::Publisher pub_markers_;
  ros::Publisher pub_markerArray_;
  ros::Publisher pub_course_;
  ros::Publisher pub_station_;

  // the messages
  visualization_msgs::Marker marker_wpt_;
  visualization_msgs::Marker marker_bulb_;
  visualization_msgs::MarkerArray marker_textArray_;
  usv16::course msg_course_;

  // the subscriber
  ros::Subscriber sub_pos_;

  // the server used to insert waypoints
  ros::ServiceServer srv_insert_;

  // the waypoints
  std::vector<double> waypoints_xcoor_;
  std::vector<double> waypoints_ycoor_;
  std::vector<double> waypoints_heading_;

  // mission information
  bool mission_type_; // true - station keeping, false - path following
  ros::Duration mission_duration_;

  // starting time of the station-keeping task
  bool station_published_ = false;
  ros::Time sk_start_time_;

  // the number of the current base waypoint
  uint32_t base_num_ = 0;

  // the waypoint to which the ship is pointed
  std::vector<double>::iterator iter_x_;
  std::vector<double>::iterator iter_y_;

  // store the position of the inserted waypoints
  std::vector<int> insert_pos_;
  // the progress of the mission (not counting inserted wpt)
  uint32_t progress_ = 0;

  // radius of the los vector
  double los_radius_ = 10;
  // radius of the capture circle
  double capture_radius_ = 8.0;

  // los guidance controlller
  guidance_law::LosGuidanceLaw los_controller_;

  // action feedback and result
  usv16::WaypointTrackingFeedback feedback_;
  usv16::WaypointTrackingResult result_;

public:
  /////////////////
  // Constructor //
  /////////////////

  WaypointTrackingServer(std::string);

  ////////////////
  // Destructor //
  ////////////////

  ~WaypointTrackingServer(void) {}

  ////////////////////
  // Public Methods //
  ////////////////////

  void goalCallback();
  void preemptCallback();
  void callback_pos(const geometry_msgs::Pose2D& msg_pos);
  bool insertCallback(usv16::waypoint::Request& req,
                      usv16::waypoint::Response& resp);
  void timerCallback(const ros::TimerEvent& event);
};

#endif // WAYPOINT_TRACKING_SERVER_H
