#include "waypoint_tracking_server.h"

#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <actionlib/server/simple_action_server.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include "usv16/course.h"
#include "usv16/waypoint.h"
#include "los_guidance_law.h"
#include "usv16/WaypointTrackingAction.h"

using namespace std;

/////////////////
// Constructor //
/////////////////

WaypointTrackingServer::WaypointTrackingServer(std::string name)
    : as_(nh_, name, false)
{
  // register the goal and feedback callbacks
  as_.registerGoalCallback(
      boost::bind(&WaypointTrackingServer::goalCallback, this));
  as_.registerPreemptCallback(
      boost::bind(&WaypointTrackingServer::preemptCallback, this));

  // the timer to control the publishing of waypoint texts to rviz
  timer_step_ = 0.1;
  timer_ = nh_.createTimer(ros::Duration(timer_step_),
                           &WaypointTrackingServer::timerCallback, this);

  // subscriber
  sub_pos_ = nh_.subscribe("ship/pose", 1000,
                           &WaypointTrackingServer::callback_pos, this);

  // publishers
  pub_course_ = nh_.advertise<usv16::course>("ship/course", 1000);
  pub_station_ = nh_.advertise<geometry_msgs::Pose2D>("ship/station", 1000);
  pub_markers_ =
      nh_.advertise<visualization_msgs::Marker>("ship/waypoints_marker", 1000);
  pub_markerArray_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "ship/waypoint_label", 1000);

  // define the service used to add waypoints
  srv_insert_ = nh_.advertiseService(
      "insert_waypoint", &WaypointTrackingServer::insertCallback, this);

  // define the controller
  los_controller_ = guidance_law::LosGuidanceLaw(los_radius_);

  // define the marker as the waypoints
  marker_wpt_.header.frame_id = "world";
  marker_wpt_.header.stamp = ros::Time::now();
  marker_wpt_.ns = "waypoints/viz";
  marker_wpt_.action = visualization_msgs::Marker::ADD;
  marker_wpt_.pose.orientation.w = 1.0;
  marker_wpt_.id = 0;
  marker_wpt_.type = visualization_msgs::Marker::LINE_LIST;
  marker_wpt_.scale.x = 0.1; // line width
  marker_wpt_.color.g = 1.0;
  marker_wpt_.color.a = 1.0;
  marker_wpt_.lifetime = ros::Duration();

  // define a marker to highlight the waypoint to which the ship is pointed
  marker_bulb_.header.frame_id = "world";
  marker_bulb_.header.stamp = ros::Time::now();
  marker_bulb_.ns = "waypoints/viz";
  marker_bulb_.action = visualization_msgs::Marker::ADD;
  marker_bulb_.id = 1;
  marker_bulb_.type = visualization_msgs::Marker::SPHERE;
  marker_bulb_.scale.x = 0.2;
  marker_bulb_.scale.y = 0.2;
  marker_bulb_.scale.z = 0.2;
  marker_bulb_.color.r = 1.0; // red
  marker_bulb_.color.a = 0.0; // invisible initially
  marker_bulb_.lifetime = ros::Duration();

  // start the action server
  as_.start();
  ROS_INFO("Waypoint-tracking server starts, waiting for a goal.");
}

////////////////////
// Public Methods //
////////////////////

void WaypointTrackingServer::goalCallback()
{
  // reset the waypoints
  waypoints_xcoor_.clear();
  waypoints_ycoor_.clear();
  marker_wpt_.points.clear();
  progress_ = 0;

  // get the goal
  auto ptr_goal = as_.acceptNewGoal();

  waypoints_xcoor_ = ptr_goal->pos_x;
  waypoints_ycoor_ = ptr_goal->pos_y;
  waypoints_heading_ = ptr_goal->heading;
  mission_type_ = ptr_goal->mission_type;
  mission_duration_ = ros::Duration(ptr_goal->mission_duration);

  // reset the iterators
  base_num_ = 0;
  insert_pos_.clear();
  iter_x_ = ++waypoints_xcoor_.begin();
  iter_y_ = ++waypoints_ycoor_.begin();

  // define the markers of waypoints using the new coordinates
  for (uint32_t i = 0; i < waypoints_xcoor_.size(); ++i)
  {
    geometry_msgs::Point p;
    p.x = waypoints_xcoor_[i];
    p.y = -waypoints_ycoor_[i];
    p.z = 0;
    marker_wpt_.points.push_back(p);
    p.z += 1.5;
    marker_wpt_.points.push_back(p);
  }

  ROS_INFO("-- New maneuver recieved --");
  ROS_INFO("The coordinates of the waypoints are:");
  ROS_INFO("wpt ----- x (m) ------- y (m)");

  for (uint32_t i = 0; i < waypoints_xcoor_.size(); ++i)
  {
    ROS_INFO("%2u ------ %5.2f ------- %5.2f", i, waypoints_xcoor_[i],
             waypoints_ycoor_[i]);
  }
}

void WaypointTrackingServer::preemptCallback()
{
  ROS_WARN("----- Current maneuver preempted -----");
  // set the action state to preempted
  result_.final_wpt = base_num_;
  as_.setPreempted(result_);
}

// callback of the subscriber, where desired course is computed and published
void WaypointTrackingServer::callback_pos(const geometry_msgs::Pose2D& msg_pos)
{
  if (!as_.isActive())
  {
    ROS_INFO_ONCE("The waypoint-tracking server is not active.");
    return;
  }

  if (base_num_ < waypoints_xcoor_.size() - 1)
  {
    array<double, 3> course_info = guidance_law::ComputeCourse(
        los_controller_, msg_pos.x, msg_pos.y, msg_pos.theta,
        waypoints_xcoor_[base_num_], waypoints_xcoor_[base_num_ + 1],
        waypoints_ycoor_[base_num_], waypoints_ycoor_[base_num_ + 1]);

    // publish the course info
    msg_course_.angle = course_info[0];
    msg_course_.rate = course_info[1];
    msg_course_.acceleration = course_info[2];
    pub_course_.publish(msg_course_);

    // the distance between the ship and the waypoint to which it points
    double distance = sqrt(pow(msg_pos.x - waypoints_xcoor_[base_num_ + 1], 2) +
                           pow(msg_pos.y - waypoints_ycoor_[base_num_ + 1], 2));

    // change the waypoint when necessary
    if (distance <= capture_radius_)
    {
      // update and publish the feedback if the reached waypoint is not newly
      // inserted
      if (find(insert_pos_.begin(), insert_pos_.end(), base_num_ + 1) ==
          insert_pos_.end())
      {
        ++progress_;
        feedback_.progress = progress_;
        as_.publishFeedback(feedback_);
      }
      ++base_num_;
      ++iter_x_;
      ++iter_y_;

      ROS_INFO("Change the base waypoint to wpt%u.", base_num_);
    }
  }
  else
  {
    if (!mission_type_) // only for waypoint-tracking missions
    {
      ROS_WARN("The vehicle has reached the last waypoint.");
      result_.final_wpt = base_num_;
      as_.setSucceeded(result_);
      ROS_WARN("------ Current maneuver completed ------");
      ROS_INFO("The action server is not active.");
    }
    else
    {
      // publish the desired station
      ROS_WARN_STREAM_THROTTLE(60,
                               "The vehicle is now in station-keeping mode.");
      geometry_msgs::Pose2D station_pos;
      station_pos.x = waypoints_xcoor_[base_num_];
      station_pos.y = waypoints_ycoor_[base_num_];
      station_pos.theta = waypoints_heading_[base_num_];
      pub_station_.publish(station_pos);

      // track the progress of the station-keeping task
      if (!station_published_)
      {
        sk_start_time_ = ros::Time::now();
        station_published_ = true;
      }

      if ((ros::Time::now() - sk_start_time_) >= mission_duration_)
      {
        ROS_WARN("Station-keeping task completed.");
        station_published_ = false;
        result_.final_wpt = base_num_;
        as_.setSucceeded(result_);
        ROS_WARN("------ Current maneuver completed ------");
        ROS_INFO("The action server is not active.");
      }
    }
  }

  // update the bulb
  if (base_num_ < waypoints_xcoor_.size() - 1)
  {
    marker_bulb_.pose.position.x = waypoints_xcoor_[base_num_ + 1];
    marker_bulb_.pose.position.y = -waypoints_ycoor_[base_num_ + 1];
    marker_bulb_.pose.position.z = 1.5;
    marker_bulb_.color.a = 1.0;
  }
  else
  {
    marker_bulb_.pose.position.x = waypoints_xcoor_[base_num_];
    marker_bulb_.pose.position.y = -waypoints_ycoor_[base_num_];
    marker_bulb_.pose.position.z = 1.5;
    marker_bulb_.color.a = 1.0;
  }
}

// callback of the service server
bool WaypointTrackingServer::insertCallback(usv16::waypoint::Request& req,
                                            usv16::waypoint::Response& resp)
{
  if (!as_.isActive())
  {
    ROS_ERROR("The action server is not active. NO waypoint added.");
    return false;
  }

  // insert the new coordinates into the vectors
  iter_x_ = waypoints_xcoor_.insert(iter_x_, req.x);
  iter_y_ = waypoints_ycoor_.insert(iter_y_, req.y);

  // remember the position of the inserted wpt
  insert_pos_.push_back(base_num_ + 1);

  ROS_INFO("Waypoint inserted as wpt%u. The coordinates of the waypoints are:",
           base_num_ + 1);
  ROS_INFO("wpt ----- x (m) ------- y (m)");

  for (uint32_t i = 0; i < waypoints_xcoor_.size(); ++i)
  {
    ROS_INFO("%2u ------ %5.2f ------- %5.2f", i, waypoints_xcoor_[i],
             waypoints_ycoor_[i]);
  }

  // define the response
  string feedback = "The waypoint is inserted as wpt";
  resp.feedback = feedback + to_string(base_num_ + 1);

  // send the new waypoint to rviz
  geometry_msgs::Point p;
  p.x = req.x;
  p.y = -req.y;
  p.z = 0;
  marker_wpt_.points.push_back(p);
  p.z += 1.5;
  marker_wpt_.points.push_back(p);

  return true;
}

// callback of the timer
void WaypointTrackingServer::timerCallback(const ros::TimerEvent& event)
{
  // define the waypoint labels
  marker_textArray_.markers.clear();

  for (uint32_t i = 0; i < waypoints_xcoor_.size(); ++i)
  {
    visualization_msgs::Marker marker_text;
    string name = "wpt";
    marker_text.header.frame_id = "world";
    marker_text.header.stamp = ros::Time::now();
    marker_text.ns = "waypoints/viz";
    marker_text.action = visualization_msgs::Marker::ADD;
    marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_text.id = i + 2;
    marker_text.lifetime = ros::Duration(timer_step_);
    marker_text.scale.z = 0.2;
    marker_text.text = name + to_string(i);
    marker_text.pose.position.x = waypoints_xcoor_[i];
    marker_text.pose.position.y = -waypoints_ycoor_[i] - 0.2;
    marker_text.pose.position.z = 1.0;
    marker_text.color.r = 1.0; // white
    marker_text.color.b = 1.0;
    marker_text.color.g = 1.0;
    marker_text.color.a = 1.0;

    marker_textArray_.markers.push_back(marker_text);
  }

  pub_markers_.publish(marker_wpt_);
  pub_markers_.publish(marker_bulb_);
  pub_markerArray_.publish(marker_textArray_);
}
