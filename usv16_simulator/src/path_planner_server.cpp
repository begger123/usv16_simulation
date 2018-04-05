#include "path_planner_server.h"

#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <visualization_msgs/MarkerArray.h>
#include "usv16/PathPlannerAction.h"

#include "path_planner/ConfigFile.h"
#include "path_planner/TrajectoryPlanner.h"
#include "path_planner/Parser.h"
#include "path_planner/Action.h"

using namespace std;

/////////////////
// Constructor //
/////////////////

PathPlannerServer::PathPlannerServer(std::string name)
    : as_(nh_, name, boost::bind(&PathPlannerServer::executeCallback, this, _1),
          false)
{
  // define the publisher
  pub_markerArray_ =
      nh_.advertise<visualization_msgs::MarkerArray>("/static_obstacles", 1000);

  // the timer to control the publishing of obstacles to rviz
  timer_step_ = 1;
  timer_ = nh_.createTimer(ros::Duration(timer_step_),
                           &PathPlannerServer::timerCallback, this);

  // define the waypoint labels
  markerArray_obstacle_.markers.clear();

  // load the obstacles
  Parser lParserObj;
  
  std::string ObstacleFile = "/home/travematics/Documents/msl_clone_cws/src/usv16_simulation/usv16_simulator/config/obstacles/ObstaclesCenter.txt";
  std::ifstream f(ObstacleFile.c_str());
  if (!f.good())
    ROS_ERROR_STREAM("ObstacleCenter file cannot be found!");
  else
  {
    ROS_INFO_STREAM("ObstacleCenter file is found.");

    vector<vector<double>> Obstacles =
        lParserObj.loadObstaclesCenter(ObstacleFile);

    for (uint32_t i = 0; i < Obstacles.size(); ++i)
    {
      visualization_msgs::Marker marker_cylinder;
      marker_cylinder.header.frame_id = "world";
      marker_cylinder.header.stamp = ros::Time::now();
      marker_cylinder.ns = "static_obstacles/viz";
      marker_cylinder.action = visualization_msgs::Marker::ADD;
      marker_cylinder.type = visualization_msgs::Marker::CYLINDER;
      marker_cylinder.id = i + 2;
      marker_cylinder.lifetime = ros::Duration();
      marker_cylinder.scale.x = 2 * Obstacles[i][2];
      marker_cylinder.scale.y = 2 * Obstacles[i][2];
      marker_cylinder.scale.z = 1;
      marker_cylinder.pose.position.x = Obstacles[i][0];
      marker_cylinder.pose.position.y = -Obstacles[i][1];
      marker_cylinder.pose.position.z = 0.0;
      marker_cylinder.color.r = 1.0; // red
      marker_cylinder.color.b = 0.0;
      marker_cylinder.color.g = 0.0;
      marker_cylinder.color.a = 0.5;

      markerArray_obstacle_.markers.push_back(marker_cylinder);
    }
  }

  // start the action server
  as_.start();
  ROS_INFO("Path planner starts, waiting for a target.");
}

////////////////////
// Public Methods //
////////////////////

void PathPlannerServer::executeCallback(
    const usv16::PathPlannerGoalConstPtr& goal)
{
  vector<double> start_pose = goal->start_pose;
  vector<double> end_pose = goal->end_pose;
  vector<double> start_vel = goal->start_vel;

  ConfigFile lCfg("/home/travematics/Documents/msl_clone_cws/src/usv16_simulation/usv16_simulator/config/Config.txt");
  TrajectoryPlanner lPlanner(lCfg);
  vector<double> lTraj =
      lPlanner.computeTrajectory(start_pose, start_vel, end_pose);

  usv16::PathPlannerResult result;

  if (lTraj.size() < 1)
  {
    ROS_ERROR("Cannot not find a trajectory...!!!");
    as_.setAborted(result);
  }
  else
  {
    for (size_t i = 0; i < lTraj.size() / 3; ++i)
    {
      result.wpt_x.push_back(lTraj.at(3 * i));
      result.wpt_y.push_back(lTraj.at(3 * i + 1));
      result.wpt_psi.push_back(lTraj.at(3 * i + 2));
    }

    // set the action state to succeeded
    ROS_INFO("A trajectory is found!!!");
    as_.setSucceeded(result);
  }
}

// callback of the timer
void PathPlannerServer::timerCallback(const ros::TimerEvent& event)
{
  pub_markerArray_.publish(markerArray_obstacle_);
}
