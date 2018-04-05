#ifndef WAYPOINT_TRACKING_MISSION_H
#define WAYPOINT_TRACKING_MISSION_H

#include <ros/ros.h>
#include <vector>

class Maneuver
{
  friend class UserInterface;

private:
  ////////////////////
  // Private Fields //
  ////////////////////

  // target position
  double goal_xcoor_;
  double goal_ycoor_;
  double goal_psi_;

  // waypoints
  std::vector<double> wpt_xcoor_;
  std::vector<double> wpt_ycoor_;
  std::vector<double> wpt_heading_;

  // ID
  std::string id_;

  // mission progress
  uint32_t progress_ = 0;

  // mission type
  // waypoint tracking -- falsoe
  // station keeping   -- true
  bool station_keeping_on_;

  // duration of the station-keeping task
  ros::Duration duration_;

  // mission description
  std::string description_;

public:
  /////////////////
  // Constructor //
  /////////////////

  Maneuver() = default;
  Maneuver(const std::vector<double>& wpt_xcoor,
           const std::vector<double>& wpt_ycoor,
           const std::vector<double>& wpt_heading,
           const std::string& mission_name, const bool& mission_type,
           const double& duration)
      : wpt_xcoor_(wpt_xcoor), wpt_ycoor_(wpt_ycoor), wpt_heading_(wpt_heading),
        id_(mission_name), station_keeping_on_(mission_type),
        duration_(duration)
  {
  }

  ////////////////////
  // Public Methods //
  ////////////////////

  // member function resetting the progress
  void reset_progress()
  {
    progress_ = 0;
    ROS_INFO("The progress of maneuver %s is reset.", id_.c_str());
  }

  void set_progress(uint32_t p)
  {
    if (p > wpt_xcoor_.size())
    {
      ROS_ERROR("set_progress failed: the progress shouldn't be larger than "
                "the waypoint number.");
      return;
    }
    progress_ = p;
  }

  std::string get_name() const { return id_; }
  uint32_t get_progress() const { return progress_; }
  size_t get_size() const { return wpt_xcoor_.size(); }
  std::vector<double> get_xcoor() const { return wpt_xcoor_; }
  std::vector<double> get_ycoor() const { return wpt_ycoor_; }
  std::vector<double> get_heading() const { return wpt_heading_; }
  bool get_type() const { return station_keeping_on_; }
  bool empty() const { return wpt_xcoor_.empty(); }
  double get_duration() const { return duration_.toSec(); }
};

#endif // WAYPOINT_TRACKING_MISSION_H
