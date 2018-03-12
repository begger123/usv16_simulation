#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "ship.h"
#include "usv16_viz/pose.h"
#include "usv16_viz/control.h"

// the publishers and messages
ros::Publisher *pubPtr_vel;
ros::Publisher *pubPtr_pos;
geometry_msgs::Twist msg_vel;
usv16_viz::pose msg_pos;

// the vehicle for the simulation
Ship *Ptr_vehicle;

// the time step of message publishing
double step_size = 0.05;

void timerCallback(const ros::TimerEvent&)
{
    // publish the motion information of the vehicle
    ROS_INFO("Time: %5.2f s, Main thrust: %5.2f N, Yaw moment: %5.2f Nm",
             Ptr_vehicle->current_time_, Ptr_vehicle->actuation_[0], Ptr_vehicle->actuation_[2]);
    msg_vel.linear.x = Ptr_vehicle->velocity_[0];
    msg_vel.linear.y = Ptr_vehicle->velocity_[1];
    msg_vel.linear.z = 0;
    msg_vel.angular.x = 0;
    msg_vel.angular.y = 0;
    msg_vel.angular.z = Ptr_vehicle->velocity_[2];

    msg_pos.x = Ptr_vehicle->position_[0];
    msg_pos.y = Ptr_vehicle->position_[1];
    msg_pos.heading = Ptr_vehicle->position_[2];

    pubPtr_vel->publish(msg_vel);
    pubPtr_pos->publish(msg_pos);

    // run a one-step simulation
    // the time step should be equal to the publishing rate to simulate the real time
    RunShip(*Ptr_vehicle, 1, step_size);
}

void callback_sub(const usv16_viz::control &msg_ctrl)
{
    Ptr_vehicle->actuation_[0] = msg_ctrl.surge;
    Ptr_vehicle->actuation_[1] = msg_ctrl.sway;
    Ptr_vehicle->actuation_[2] = msg_ctrl.yaw;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ship_dynamics");
    ros::NodeHandle nh;

    // define parameters for the ship's initial position
    const std::string PARAM_1 = "~init_x";
    const std::string PARAM_2 = "~init_y";
    const std::string PARAM_3 = "~init_heading";
    double init_x, init_y, init_heading;
    bool ok_1 = ros::param::get(PARAM_1, init_x);
    bool ok_2 = ros::param::get(PARAM_2, init_y);
    bool ok_3 = ros::param::get(PARAM_3, init_heading);

    if ( ok_1 && ok_2 && ok_3 )
    {
        Ptr_vehicle = new Ship({0, 0, 0}, {init_x, init_y, init_heading});
    }
    else
    {
        Ptr_vehicle = new Ship();
    }

    // create a timer to run a one-step simulation and publish messages
    ros::Timer timer = nh.createTimer(ros::Duration(step_size), &timerCallback);

    // the publishing rate is controlled by the timer
    pubPtr_vel = new ros::Publisher(nh.advertise<geometry_msgs::Twist>("ship/vel", 1000));
    pubPtr_pos = new ros::Publisher(nh.advertise<usv16_viz::pose>("ship/pose", 1000));

    // create a subscriber to the control signal
    ros::Subscriber sub_ctrl = nh.subscribe("ship/actuation", 1000, &callback_sub);

    ros::spin();

    delete pubPtr_vel;
    delete pubPtr_pos;
    delete Ptr_vehicle;

    return 0;
}
