#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gps_publisher");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher gps_pub =
      n.advertise<sensor_msgs::NavSatFix>("gps_publisher", 1);

  while (ros::ok())
  {
    sensor_msgs::NavSatFix gps;

    gps.header.frame_id = "world";

    bool ok_1 =
        ros::param::get("/ctrl_params/positionReference/latRef", gps.latitude);
    bool ok_2 =
        ros::param::get("/ctrl_params/positionReference/lonRef", gps.longitude);

    if (ok_1 && ok_2)
    {
      ROS_INFO_ONCE("The reference point is lat: %5.6f, lon: %5.6f.",
                    gps.latitude, gps.longitude);
    }
    else
    {
      ROS_ERROR("The reference point cannot be loaded!!");
    }

    gps_pub.publish(gps);

    r.sleep();
  }
}
