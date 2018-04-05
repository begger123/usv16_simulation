#include <ros/ros.h>
#include "user_interface.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "task_manager");

  // create the user interface
  UserInterface user_interface;

  return user_interface.run();
}
