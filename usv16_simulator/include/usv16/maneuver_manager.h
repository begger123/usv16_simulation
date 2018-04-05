#ifndef WAYPOINT_TRACKING_MISSION_MANAGER_H
#define WAYPOINT_TRACKING_MISSION_MANAGER_H

#include <vector>
#include <deque>
#include "maneuver.h"

class ManeuverManager
{
public:
  ///////////////////
  // Public Fields //
  ///////////////////

  std::map<std::string, Maneuver> mission_database_;
  std::set<std::string> mission_set_;
  std::deque<std::string> mission_queue_;

  /////////////////
  // Constructor //
  /////////////////

  ManeuverManager() = default;
};

#endif // WAYPOINT_TRACKING_MISSION_MANAGER_H
