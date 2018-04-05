#include <iostream>
#include <ros/ros.h>
#include "usv16/control.h"

class ShipDriver
{
private:
  // The node handle
  ros::NodeHandle nh_;
  // publish to the "/ship/manual_control" topic to issue commands
  ros::Publisher cmd_actuation_pub_;

public:
  // constructor
  ShipDriver(ros::NodeHandle &nh)
  {
    nh_ = nh;
    //set up the publisher for the cmd_actuation topic
    cmd_actuation_pub_ = nh_.advertise<usv16::control>("/ship/manual_control", 1000);
  }

  // Loop forever while sending drive commands based on keyboard input
  bool driveKeyboard()
  {
    std::cout << "Type a command and then press enter.  "
      "Use 'w' to move forward, 'a' to turn left, "
      "'d' to turn right, 's' to stop.\n";

    // the ship actuation commands
    usv16::control ship_cmd;

    char cmd[50];
    while(nh_.ok()){

      std::cin.getline(cmd, 50);
      if(cmd[0]!='w' && cmd[0]!='s' && cmd[0]!='d' && cmd[0]!='a')
      {
        std::cout << "unknown command:" << cmd << "\n";
        continue;
      }

      ship_cmd.surge = ship_cmd.sway = ship_cmd.yaw = 0;

      // move forward
      if(cmd[0]=='w'){
        ship_cmd.surge = 1.0;
        ship_cmd.yaw = 0.0;
      }
      // turn left (yaw) and drive forward at the same time
      else if(cmd[0]=='a'){
        ship_cmd.surge = 0.2;
        ship_cmd.sway = 0.1/0.6;
        ship_cmd.yaw = -0.1;
      }
      // turn right (yaw) and drive forward at the same time
      else if(cmd[0]=='d'){
        ship_cmd.surge = 0.2;
        ship_cmd.sway = -0.1/0.6;
        ship_cmd.yaw = 0.1;
      }
      // stop the ship
      else if(cmd[0]=='s'){
        ship_cmd.surge = 0.0;
        ship_cmd.yaw = 0.0;
      }

      //publish the assembled command
      cmd_actuation_pub_.publish(ship_cmd);
    }
    return true;
  }

};

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "ship_driver");
  ros::NodeHandle nh;

  ShipDriver driver(nh);
  driver.driveKeyboard();
}
