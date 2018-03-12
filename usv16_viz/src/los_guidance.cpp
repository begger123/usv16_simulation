#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <boost/numeric/odeint.hpp>
#include <boost/math/special_functions/sign.hpp>
#include <actionlib/server/simple_action_server.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include "usv16_viz/pose.h"
#include "usv16_viz/course.h"
#include "usv16_viz/waypoint.h"
#include "los_guidance_law.h"
#include "usv16_viz/WaypointTrackingAction.h"

using namespace std;
using namespace boost::numeric::odeint;


class WaypointTrackingServer
{
protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<usv16_viz::WaypointTrackingAction> as_;

    // the timer which controls the publication of markers to rviz
    ros::Timer timer_;

    // the publishers
    ros::Publisher pub_markers_;
    ros::Publisher pub_markerArray_;
    ros::Publisher pub_course_;

    // the messages
    visualization_msgs::Marker marker_wpt_;
    visualization_msgs::Marker marker_bulb_;
    visualization_msgs::MarkerArray marker_textArray_;
    usv16_viz::course msg_course_;

    // the subscriber
    ros::Subscriber sub_pos_;

    // the server
     ros::ServiceServer srv_insert_;

    // the waypoints
    vector<double> waypoints_xcoor_;
    vector<double> waypoints_ycoor_;

    // the number of the base waypoint
    uint32_t base_num_ = 0;

    // the waypoint to which the ship is pointed
    vector<double>::iterator iter_x_;
    vector<double>::iterator iter_y_;

    // store the position of the inserted waypoints
    vector<int> insert_pos_;
    // the progress of the mission (not counting inserted wpt)
    uint32_t progress_ = 0;

    // generated course infomation
    vector<double> course_state_ = {0, 0, 0};

    // radius of the capture circle
    double radius_ = 1.0;

    // los guidance controlller
    los_guidance_law::LosGuidanceLaw los_controller_;

    // action feedback and result
    usv16_viz::WaypointTrackingFeedback feedback_;
    usv16_viz::WaypointTrackingResult result_;

public:
    WaypointTrackingServer(std::string name):
        as_(nh_, name, false)
    {
        // register the goal and feedback callbacks
        as_.registerGoalCallback(boost::bind(&WaypointTrackingServer::goalCB, this));
        as_.registerPreemptCallback(boost::bind(&WaypointTrackingServer::preemptCB, this));

        // the timer to control the publishing to rviz
        timer_ = nh_.createTimer(ros::Duration(0.1), &WaypointTrackingServer::timerCallback, this);

        // subscriber
        sub_pos_ = nh_.subscribe("ship/pose", 1000, &WaypointTrackingServer::callback_pos, this);

        // publishers
        pub_course_ = nh_.advertise<usv16_viz::course>("ship/course", 1000);
        pub_markers_ = nh_.advertise<visualization_msgs::Marker>("ship/waypoints", 1000);
        pub_markerArray_ = nh_.advertise<visualization_msgs::MarkerArray>("ship/waypoint_label", 1000);

        // define the service used to add waypoints
        srv_insert_ = nh_.advertiseService("insert_waypoint", &WaypointTrackingServer::insertCallback, this);

        // define the marker as the waypoints
        marker_wpt_.header.frame_id = "world";
        marker_wpt_.header.stamp = ros::Time::now();
        marker_wpt_.ns = "waypoints/viz";
        marker_wpt_.action = visualization_msgs::Marker::ADD;
        marker_wpt_.pose.orientation.w = 1.0;
        marker_wpt_.id = 0;
        marker_wpt_.type = visualization_msgs::Marker::LINE_LIST;
        marker_wpt_.scale.x = 0.1; // line width
        marker_wpt_.color.g = 1.0;
        marker_wpt_.color.a = 1.0;
        marker_wpt_.lifetime = ros::Duration();

        // define a marker to highlight the waypoint to which the ship is pointed
        marker_bulb_.header.frame_id = "world";
        marker_bulb_.header.stamp = ros::Time::now();
        marker_bulb_.ns = "waypoints/viz";
        marker_bulb_.action = visualization_msgs::Marker::ADD;
        marker_bulb_.id = 1;
        marker_bulb_.type = visualization_msgs::Marker::SPHERE;
        marker_bulb_.scale.x = 0.2;
        marker_bulb_.scale.y = 0.2;
        marker_bulb_.scale.z = 0.2;
        marker_bulb_.color.r = 1.0; // red
        marker_bulb_.color.a = 1.0;
        marker_bulb_.lifetime = ros::Duration();

        // start the action server
        as_.start();
        ROS_INFO("Server starts, waiting for a goal.");
    }

    ~WaypointTrackingServer(void) {}

    void goalCB()
    {
        // reset the waypoints
        waypoints_xcoor_.clear();
        waypoints_ycoor_.clear();
        marker_wpt_.points.clear();
        progress_ = 0;

        auto ptr_goal = as_.acceptNewGoal();

        waypoints_xcoor_ = ptr_goal->pos_x;
        waypoints_ycoor_ = ptr_goal->pos_y;
        course_state_[0] = ptr_goal->init_heading;

        // initialize the controller
        los_controller_ = los_guidance_law::LosGuidanceLaw(course_state_, radius_, 0.05);

        // reset the iterators
        base_num_ = 0;
        insert_pos_.clear();
        iter_x_ = ++waypoints_xcoor_.begin();
        iter_y_ = ++waypoints_ycoor_.begin();

        // define the markers of waypoints using the new coordinates
        for (uint32_t i = 0; i < waypoints_xcoor_.size(); ++i)
        {
            geometry_msgs::Point p;
            p.x = waypoints_xcoor_[i];
            p.y = -waypoints_ycoor_[i];
            p.z = 0;
            marker_wpt_.points.push_back(p);
            p.z += 1.5;
            marker_wpt_.points.push_back(p);
        }

        ROS_INFO("-------- New mission recieved --------");
        ROS_INFO("The coordinates of the waypoints are:");
        ROS_INFO("wpt ----- x (m) ------- y (m)");

        for (uint32_t i = 0; i < waypoints_xcoor_.size(); ++i)
        {
            ROS_INFO("%2u ------ %5.2f ------- %5.2f",
                     i, waypoints_xcoor_[i],  waypoints_ycoor_[i]);
        }
    }

    void preemptCB()
    {
        ROS_WARN("----- Current mission preempted -----");
        // set the action state to preempted
        result_.final_wpt = base_num_;
        as_.setPreempted(result_);
    }

    void callback_pos(const usv16_viz::pose &msg_pos);
    bool insertCallback(usv16_viz::waypoint::Request &req, usv16_viz::waypoint::Response &resp);
    void timerCallback(const ros::TimerEvent &event);

};

// callback of the subscriber, where desired course is computed and published
void WaypointTrackingServer::callback_pos(const usv16_viz::pose &msg_pos)
{
    if (!as_.isActive()) {
        ROS_INFO_ONCE("The action server is not active.");
        return; }

    if (base_num_ < waypoints_xcoor_.size()-1)
    {
        array<double, 3> course_info = los_guidance_law::ComputeCourse(los_controller_, msg_pos.x, msg_pos.y, msg_pos.heading,
                                                    waypoints_xcoor_[base_num_], waypoints_xcoor_[base_num_+1],
                                                    waypoints_ycoor_[base_num_], waypoints_ycoor_[base_num_+1]);

        // publish the course info
        msg_course_.angle = course_info[0];
        msg_course_.rate = course_info[1];
        msg_course_.acceleration = course_info[2];
        pub_course_.publish(msg_course_);

        // the distance between the ship and the waypoint to which it points
        double distance = sqrt(pow(msg_pos.x - waypoints_xcoor_[base_num_+1], 2) +
                               pow(msg_pos.y - waypoints_ycoor_[base_num_+1], 2));
//        ROS_INFO("Base: wpt%u, Distance to wpt%u: %5.2f m", wpt_num_, wpt_num_+1, distance);

        // change the waypoint if necessary
        if ( distance <= radius_)
        {
            // update and publish the feedback
            if (find(insert_pos_.begin(), insert_pos_.end(), base_num_+1) == insert_pos_.end())
            {
                ++ progress_;
                feedback_.progress = progress_;
                as_.publishFeedback(feedback_);
            }
            ++ base_num_;
            ++ iter_x_;
            ++ iter_y_;

            ROS_INFO("Change the base waypoint to wpt%u.", base_num_);
        }
    }
    else
    {
        ROS_WARN("The ship has reached the last waypoint.");
        result_.final_wpt = base_num_;
        as_.setSucceeded(result_);
        ROS_WARN("------ Current mission completed ------");
        ROS_INFO("The action server is not active.");
    }


    // publish the markers to rviz
    if (base_num_ < waypoints_xcoor_.size()-1) {
        marker_bulb_.pose.position.x = waypoints_xcoor_[base_num_+1];
        marker_bulb_.pose.position.y = -waypoints_ycoor_[base_num_+1];
        marker_bulb_.pose.position.z = 1.5;
    }
}

// callback of the service server
bool WaypointTrackingServer::insertCallback(usv16_viz::waypoint::Request &req, usv16_viz::waypoint::Response &resp)
{
    if (!as_.isActive()) {
        ROS_ERROR("The action server is not active. NO waypoint added.");
        return false; }

    // insert the new coordinates into the vectors
    iter_x_ = waypoints_xcoor_.insert(iter_x_, req.x);
    iter_y_ = waypoints_ycoor_.insert(iter_y_, req.y);

    // remember the position of the inserted wpt
    insert_pos_.push_back(base_num_+1);

    ROS_INFO("Waypoint inserted as wpt%u. The coordinates of the waypoints are:", base_num_+1);
    ROS_INFO("wpt ----- x (m) ------- y (m)");

    for (uint32_t i = 0; i < waypoints_xcoor_.size(); ++i)
    {
        ROS_INFO("%2u ------ %5.2f ------- %5.2f",
                 i, waypoints_xcoor_[i],  waypoints_ycoor_[i]);
    }

    // define the response
    string feedback = "The waypoint is inserted as wpt";
    resp.feedback =  feedback + to_string(base_num_+1);

    // send the new waypoint to rviz
    geometry_msgs::Point p;
    p.x = req.x;
    p.y = -req.y;
    p.z = 0;
    marker_wpt_.points.push_back(p);
    p.z += 1.5;
    marker_wpt_.points.push_back(p);

    return true;
}

// callback of the timer
void WaypointTrackingServer::timerCallback(const ros::TimerEvent &event)
{
    // define the waypoint labels
    marker_textArray_.markers.clear();

    for (uint32_t i = 0; i < waypoints_xcoor_.size(); ++i)
    {
        visualization_msgs::Marker marker_text;
        string name = "wpt";
        marker_text.header.frame_id = "world";
        marker_text.header.stamp = ros::Time::now();
        marker_text.ns = "waypoints/viz";
        marker_text.action = visualization_msgs::Marker::ADD;
        marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker_text.id = i+2;
        marker_text.scale.z = 0.2;
        marker_text.text = name + to_string(i);
        marker_text.pose.position.x = waypoints_xcoor_[i];
        marker_text.pose.position.y = -waypoints_ycoor_[i]-0.2;
        marker_text.pose.position.z = 1.0;
        marker_text.color.r = 1.0; // white
        marker_text.color.b = 1.0;
        marker_text.color.g = 1.0;
        marker_text.color.a = 1.0;

        marker_textArray_.markers.push_back(marker_text);
    }

    pub_markers_.publish(marker_wpt_);
    pub_markers_.publish(marker_bulb_);
    pub_markerArray_.publish(marker_textArray_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "guidance_node");

    WaypointTrackingServer waypoint_tracking(ros::this_node::getName());

    ros::spin();

    return 0;
}
