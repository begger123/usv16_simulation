#include <iostream>
#include <time.h>
#include "ros/ros.h"
#include "rosgraph_msgs/Clock.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "the clock");
	ros::NodeHandle nh;
	ros::Rate loop_rate(10);
	ros::Publisher clock_pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 10);


	while(ros::ok())
	{
		ros::Time time;
		time.init();
		time = ros::Time::now();

		//ROS_INFO("the time is: %f", time.toSec());

		rosgraph_msgs::Clock time_to_pub;
		time_to_pub.clock = time;

		clock_pub.publish(time_to_pub);

		loop_rate.sleep();
	}
	return 0;
}
