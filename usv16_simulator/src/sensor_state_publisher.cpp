#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "usv16/Usv16State.h"
#include <geometry_msgs/Pose2D.h>

// declare the publishers and messages
ros::Publisher pub_vel, pub_pose;
geometry_msgs::Pose2D msg_pose;
geometry_msgs::Twist msg_vel;

// callback function of the subscriber to /ship/state
void SubCallback(const usv16::Usv16StateConstPtr& msg)
{
  msg_pose.x = msg->pose.x;
  msg_pose.y = msg->pose.y;
  msg_pose.theta = msg->pose.theta;
  msg_vel = msg->vel;

  // publish the messages
  pub_vel.publish(msg_vel);
  pub_pose.publish(msg_pose);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "state_publisher");
  ros::NodeHandle nh;

  // define the subscriber
  ros::Subscriber sub_state = nh.subscribe("/ship/state", 10, &SubCallback);

  // define the publishers
  ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("/ship/vel", 10);
  ros::Publisher pub_pose = nh.advertise<geometry_msgs::Pose2D>("/ship/pose", 10);

  ros::spin();
}