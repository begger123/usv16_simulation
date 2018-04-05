#ifndef VIZ_TRAJECTORY_H
#define VIZ_TRAJECTORY_H

#include <memory>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose2D.h>

/////////////////////////
// Forward Declaration //
/////////////////////////

namespace ros
{

class NodeHandle;
class Subscriber;
class Publisher;
}

namespace tf
{

class TransformBroadcaster;
}

class VizTrajectory
{
public:
  /////////////////
  // Constructor //
  /////////////////

  VizTrajectory();

  ////////////////////
  // Public Methods //
  ////////////////////

  int run();

  ////////////////
  // Destructor //
  ////////////////

  ~VizTrajectory();

private:
  ////////////////////
  // Private Fields //
  ////////////////////

  // Ros node and pulisher/subscriber
  std::unique_ptr<ros::NodeHandle> nh_;
  std::unique_ptr<ros::Subscriber> sub_pos_; // position subscriber
  std::unique_ptr<ros::Publisher> pub_traj_; // trajectory publisher
  std::unique_ptr<ros::Publisher> pub_ship_; // ship marker publisher

  // rviz visualization msg
  visualization_msgs::Marker trajectory_;
  visualization_msgs::Marker ship_;

  // coordinate transformation
  std::unique_ptr<tf::TransformBroadcaster>
      br_ef_; // the earth-fixed frame broadcaster
  std::unique_ptr<tf::TransformBroadcaster>
      br_bf_; // the body-fixed frame broadcaster

  /////////////////////
  // Private methods //
  /////////////////////

  void sub_callback(const geometry_msgs::Pose2DConstPtr& msg);
};

#endif // VIZ_TRAJECTORY_H
