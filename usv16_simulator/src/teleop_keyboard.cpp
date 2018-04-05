#include <ros/ros.h>
#include <usv16/control.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

class TeleopShip
{
public:
  TeleopShip();
  void keyLoop();

private:
  ros::NodeHandle nh_;
  double surge_, yaw_;
  ros::Publisher control_pub_;
};

TeleopShip::TeleopShip() : surge_(0), yaw_(0)
{
  control_pub_ = nh_.advertise<usv16::control>("ship/manual_control", 1000);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_ship");
  TeleopShip teleop_ship;

  signal(SIGINT, quit);

  teleop_ship.keyLoop();

  return (0);
}

void TeleopShip::keyLoop()
{
  char c;
  bool dirty = false;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to steer the ship.");
  puts("     up  --  move forward");
  puts("   left  --  turn left");
  puts("  right  --  turn right");
  puts("   down  --  stop");

  for (;;)
  {
    // get the next event from the keyboard
    if (read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    surge_ = yaw_ = 0;
    ROS_DEBUG("value: 0x%02X\n", c);

    switch (c)
    {
    case KEYCODE_L:
      ROS_DEBUG("LEFT");
      surge_ = 100;
      yaw_ = -60;
      dirty = true;
      break;
    case KEYCODE_R:
      ROS_DEBUG("RIGHT");
      surge_ = 100;
      yaw_ = 60;
      dirty = true;
      break;
    case KEYCODE_U:
      ROS_DEBUG("UP");
      surge_ = 200;
      dirty = true;
      break;
    case KEYCODE_D:
      ROS_DEBUG("DOWN");
      surge_ = 0;
      dirty = true;
      break;
    }

    usv16::control control;
    control.surge = surge_;
    control.yaw = yaw_;
    control.sway = 0;

    if (dirty == true)
    {
      control_pub_.publish(control);
      dirty = false;
    }
  }

  return;
}
