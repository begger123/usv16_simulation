#include "viz_trajectory.h"
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <cstdlib>
#include <geometry_msgs/Pose2D.h>
#include "usv16_msgs/Usv16State.h"

using namespace std;

/////////////////
// Constructor //
/////////////////

VizTrajectory::VizTrajectory()
{
  // populate node handle
  nh_ = unique_ptr<ros::NodeHandle>(new ros::NodeHandle);

  // define the subscriber/publisher
  ROS_WARN("HOTFIX");
  sub_pos_ = unique_ptr<ros::Subscriber>(new ros::Subscriber(
      nh_->subscribe("ship/state", 1000, &VizTrajectory::sub_callback, this)));
  pub_traj_ = unique_ptr<ros::Publisher>(
      new ros::Publisher(nh_->advertise<visualization_msgs::Marker>(
          "ship/trajectory_marker", 1000)));
  pub_ship_ = unique_ptr<ros::Publisher>(new ros::Publisher(
      nh_->advertise<visualization_msgs::Marker>("ship/ship_marker", 10)));

  // initialize the transformations
  br_ef_ = unique_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster);
  br_bf_ = unique_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster);

  // initialize the trajectory marker
  trajectory_.header.frame_id = "world";
  trajectory_.header.stamp = ros::Time::now();
  trajectory_.ns = "ship/viz";
  trajectory_.action = visualization_msgs::Marker::ADD;
  trajectory_.pose.orientation.w = 1.0;
  trajectory_.id = 0;
  trajectory_.type = visualization_msgs::Marker::LINE_STRIP;
  trajectory_.scale.x = 0.05; // line width
  trajectory_.color.r = 1.0;
  trajectory_.color.g = 0.0;
  trajectory_.color.b = 1.0;
  trajectory_.color.a = 1.0;

  // initialize the ship marker
  ship_.header.frame_id = "world";
  ship_.header.stamp = ros::Time::now();
  ship_.ns = "ship/viz";
  ship_.action = visualization_msgs::Marker::ADD;
  ship_.pose.orientation.w = 1.0;
  ship_.id = 0;
  ship_.type = visualization_msgs::Marker::TRIANGLE_LIST;
  ship_.scale.x = 1;
  ship_.scale.y = 1;
  ship_.scale.z = 1;
  ship_.color.r = 1.0;
  ship_.color.g = 1.0;
  ship_.color.b = 0.0;
  ship_.color.a = 0.3;
}

////////////////
// Destructor //
////////////////

VizTrajectory::~VizTrajectory() {}

////////////////////
// Public Methods //
////////////////////

int VizTrajectory::run()
{
  // control the publishing rate
  ros::Rate r(10);

  while (ros::ok())
  {
    pub_traj_->publish(trajectory_);
    pub_ship_->publish(ship_);

    ros::spinOnce();
    r.sleep();
  }

  return EXIT_FAILURE;
}

/////////////////////
// Private methods //
/////////////////////

void VizTrajectory::sub_callback(const usv16_msgs::Usv16State::ConstPtr& stateMsg)
{
  geometry_msgs::Point p;

  geometry_msgs::Pose2D msg = stateMsg->pose;
  // update the points of the line_strip
  p.x = msg.x;
  p.y = msg.y; // add the minus sign because of the transformation from world
                 // to the earth frame
  p.z = 0;

  // limit the length of trajectory
  if (trajectory_.points.size() > 2000)
  {
    trajectory_.points.erase(trajectory_.points.begin());
  }

  // update the trajectory
  trajectory_.points.push_back(p);

  // update the ship marker
  geometry_msgs::Point p1, p2;
  p.x = msg.x + 5 * cos(msg.theta);
  p.y = -(msg.y + 5 * sin(msg.theta));
  p1.x = msg.x - 3 * cos(msg.theta) - 5 * sin(msg.theta);
  p1.y = -(msg.y - 3 * sin(msg.theta) + 5 * cos(msg.theta));
  p2.x = msg.x - 3 * cos(msg.theta) + 5 * sin(msg.theta);
  p2.y = -(msg.y - 3 * sin(msg.theta) - 5 * cos(msg.theta));

  if (ship_.points.empty())
  {
    ship_.points.push_back(p);
    ship_.points.push_back(p1);
    ship_.points.push_back(p2);
  }
  else
  {
    ship_.points[0] = p;
    ship_.points[1] = p1;
    ship_.points[2] = p2;
  }

  tf::Transform transform;
  tf::Quaternion q;

  transform.setOrigin(tf::Vector3(msg.x, msg.y, 0));
  q.setRPY(0, 0, msg.theta);
  transform.setRotation(q);
  br_bf_->sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                             "Earth_fixed_frame", "Ship"));
}
