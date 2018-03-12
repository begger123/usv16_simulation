#include "usv16_viz/usv16_viz.h"

#include <cstdlib>
#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>

using namespace std;

//////////////////////
// Global Variables //
/////////////////////

static bool n_msg_ = false;
static bool start_ = false;

/////////////////
// Constructor //
/////////////////

Usv16Viz::Usv16Viz(int argc, char **argv)
{
	// initialize ros node
	ros::init(argc,argv,"viz_trajectory");

	// populate node handle
	nh_ = unique_ptr<ros::NodeHandle>(new ros::NodeHandle);

	// Setup subsribers and publishers
	st_sub_ = unique_ptr<ros::Subscriber>(new ros::Subscriber(
		nh_->subscribe("ship/state", 10, 
		&Usv16Viz::state_callback_, this)));

	traj_pub_ = unique_ptr<ros::Publisher>(new ros::Publisher(
		nh_->advertise<visualization_msgs::Marker>("viz/trajectory", 10)));

	// set up transformations
	br_ef_ = unique_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());
	br_bf_ = unique_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());

	// setup viz markers
	initialize_trajectory_marker_();
	initialize_waypoint_marker_();

	// send an initial transform to rviz
	broadcast_transforms_();
}

////////////////
// Destructor //
////////////////

Usv16Viz::~Usv16Viz()
{

}

////////////////////
// Public Methods //
////////////////////

int Usv16Viz::run()
{
	// asynchronous transmission between the two
    while (ros::ok()) {

        // new message is successful
        if(n_msg_)
        {
        	// update the trajectory
        	update_trajectory_marker_(st_msg_);

        	// broadcast the transformations
        	broadcast_transforms_(st_msg_);

            // publish visualizer message
            traj_pub_->publish(trajectory_);
            n_msg_ = false;
        }

        ros::spinOnce();
    }

    return EXIT_FAILURE;
}

/////////////////////
// Private Methods //
/////////////////////

void Usv16Viz::state_callback_(const usv16_msgs::Usv16StateConstPtr& msg)
{	
	// set pointer equal to the incoming pointer
	st_msg_ = msg;
	n_msg_ = true;
}

void Usv16Viz::broadcast_transforms_(void)
{
	// initialize transformation and quat class
	tf::Transform transform;
	tf::Quaternion q;

	transform.setOrigin(tf::Vector3(0, 0, 0));
    q.setRPY(M_PI, 0, 0);
    transform.setRotation(q);
    br_ef_->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "Earth-fixed frame"));

    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    q.setRPY(0.0, 0.0, 0.0);
    transform.setRotation(q);
    br_bf_->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "Earth-fixed frame", "Ship"));
}

void Usv16Viz::broadcast_transforms_(const usv16_msgs::Usv16StateConstPtr& msg)
{

	// initialize transformation and quat class
	tf::Transform transform;
	tf::Quaternion q;

	transform.setOrigin(tf::Vector3(0, 0, 0));
    q.setRPY(M_PI, 0, 0);
    transform.setRotation(q);
    br_ef_->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "Earth-fixed frame"));

    transform.setOrigin(tf::Vector3(msg->pose.x, msg->pose.y, 0));
    q.setRPY(0, 0, msg->pose.theta);
    transform.setRotation(q);
    br_bf_->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "Earth-fixed frame", "Ship"));
}

void Usv16Viz::update_trajectory_marker_(const usv16_msgs::Usv16StateConstPtr& msg)
{	
	// create point equal to current position of vehicle
	geometry_msgs::Point p; 
	p.x = msg->pose.x;
	p.y = -msg->pose.y;
	p.z = 0.0;

	// limit the length of trajectory
    if (trajectory_.points.size() > 2000)
        trajectory_.points.erase(trajectory_.points.begin());

    trajectory_.points.push_back(p);
}


void Usv16Viz::initialize_trajectory_marker_(void)
{	
	// define the marker as the trajectory
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
}

void Usv16Viz::initialize_waypoint_marker_(void)
{

}