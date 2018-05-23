/**
 * @file usv16_ros_interface.h
 * @author Ivan Bertaska
 * @date 29 Jun 2016
 * @brief File containing example of doxygen usage for quick reference.
 *
 * Here typically goes a more extensive explanation of what the header
 * defines. Doxygens tags are words preceeded by either a backslash @\
 * or by an at symbol @@.
 */


#include "usv16_sim/usv16_ros_interface.h"

#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <signal.h>
#include <vector>
#include <string>

#include "ros/ros.h"
#include "ros/transport_hints.h" 	// <- udp hints to use udp
#include "usv16_msgs/Usv16State.h"
#include "geometry_msgs/Pose.h"
#include "gazebo_msgs/SetModelState.h"


#define EPSILON_NSEC 1

using namespace std;

// Global variables limited in scope
static bool n_msg_ 			= false;
static bool fst_msg_ 		= false;
static const string db_name = "USV16-ROS Inteface : ";

class eff2world
{
public:
  eff2world() : tf_(),  target_frame_("Earth-fixed frame")
  {
    pose_sub_.subscribe(n_, "/ship/pose", 10);
    tf_filter_ = new tf::MessageFilter<geometry_msgs::PoseStamped>(pose_sub_, tf_, target_frame_, 10);
    tf_filter_->registerCallback( boost::bind(&eff2world::msgCallback, this, _1) );
  } ;
  geometry_msgs::PoseStamped state_out;
  void printTheState()
  {
  	ROS_WARN("The written position is:%f, %f, %f.",state_out.pose.position.x,state_out.pose.position.y,state_out.pose.position.z);
	ROS_WARN("The written orientation is:%f, %f, %f, %f.",state_out.pose.orientation.x,state_out.pose.orientation.y,state_out.pose.orientation.z,state_out.pose.orientation.w);
  };
private:
  message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub_;
  tf::TransformListener tf_;
  tf::MessageFilter<geometry_msgs::PoseStamped> * tf_filter_;
  ros::NodeHandle n_;
  std::string target_frame_;

  //  Callback to register with tf::MessageFilter to be called when transforms are available
  void msgCallback(const boost::shared_ptr<const geometry_msgs::PoseStamped>& pose_ptr) 
  {
    try 
    {
      tf_.transformPose(target_frame_, *pose_ptr, state_out);
    }
    catch (tf::TransformException &ex) 
    {
      printf ("Failure %s\n", ex.what()); //Print exception which was caught
    }
  };
};

Usv16RosInterface::Usv16RosInterface(int argc ,char ** argv)
{
	// initialize ros
	ros::init(argc, argv, "usv16_node_sim");

	// setup signal interrupt handler
	signal(SIGINT, Usv16RosInterface::signal_int_handler_);

	// populate node handles
	act_nh_ = unique_ptr<ros::NodeHandle>(new ros::NodeHandle);
	st_nh_ 	= unique_ptr<ros::NodeHandle>(new ros::NodeHandle);

	/**	
	 *	NOTE:: For whatever reason, 'rostopic pub' does NOT work with UDP. For that reason, 
	 *	UDP support is currently disabled until a better test system can be built.
	 */

	// subscribe to actuator topic, only keep last 10 readings, using callback method refering to THIS object
	sub_ = unique_ptr<ros::Subscriber>(new ros::Subscriber(
		act_nh_->subscribe("ship/actuation", 10, 
		&Usv16RosInterface::actuator_callback_, this)));
		// &Usv16RosInterface::actuator_callback_, this, ros::TransportHints().udp())));

	st_pub_ = unique_ptr<ros::Publisher>(new ros::Publisher(
		st_nh_->advertise<usv16_msgs::Usv16State>("ship/state", 10)));

	clk_pub_ = unique_ptr<ros::Publisher>(new ros::Publisher(
		st_nh_->advertise<rosgraph_msgs::Clock>("/clock", 10)));

	pose_pub = st_nh_->advertise<geometry_msgs::PoseStamped>("/ship/pose", 10);

	ROS_INFO_STREAM(db_name << "STARTED");
}

int Usv16RosInterface::simulate(void)
{

	// Sanity checks to make sure actuator simulation and dynamic simulation were
	// defined

	// create a ROS service to handle reseting - only exists in this scope
	ros::NodeHandle 	srv_nh;
	ros::ServiceServer	srv = srv_nh.advertiseService("sim/reset",
		&Usv16RosInterface::reset_service_callback_, this);

	// populate initial conditions
	populate_ini_sim_();

	usv16_msgs::Usv16ActuatorInputsPtr 	null_msg( new usv16_msgs::Usv16ActuatorInputs() );

	null_msg->actuator_inputs[0] = 0.0;
	null_msg->actuator_inputs[1] = 0.0;
	null_msg->actuator_inputs[2] = 0.0;
	null_msg->actuator_inputs[3] = 0.0;

	// set pointer to this null_msg
	act_msg_ = null_msg;

	// // while there is no first message, at a consistent rate with initial conditions
	while(false == fst_msg_)
	{
		// continue incrementing time until first message is received, then 
		// only publish every time an actuator is sent
		st_pub_->publish(simulate_topic_wrapper_(act_msg_));
		
		// publish clock server
		clk_pub_->publish(conv_time_to_topic_(get_prv_state()));

		// non-ROS sleep command
		usleep(get_ts()*1e6L);

		ros::spinOnce();
	}

	//used to publish to gazebo
	eff2world e2w;

	// enter simulation super loop
	while(ros::ok())
	{

		// If there is a new message perform calculation
		if(n_msg_)
		{
			// simulate
			usv16_msgs::Usv16State state = simulate_topic_wrapper_(act_msg_);
			st_pub_->publish(state);
			pose_stamped.header.stamp=ros::Time::now();
			pose_stamped.header.frame_id = "world";
			pose_stamped.pose.position.x=state.pose.x;
			pose_stamped.pose.position.y=state.pose.y;
			pose_stamped.pose.position.z=0;
			pose_stamped.pose.orientation=tf::createQuaternionMsgFromYaw(state.pose.theta);
			pose_pub.publish(pose_stamped);

			geometry_msgs::Pose start_pose;
			start_pose.position = e2w.state_out.pose.position;
			start_pose.orientation = e2w.state_out.pose.orientation;

			//e2w.printTheState();

			geometry_msgs::Twist start_twist;
			start_twist.linear.x = 0.0;
			start_twist.linear.y = 0.0;
			start_twist.linear.z = 0.0;
			start_twist.angular.x = 0.0;
			start_twist.angular.y = 0.0;
			start_twist.angular.z = 0.0;

			gazebo_msgs::ModelState modelstate;
			modelstate.model_name = (std::string) "ship";
			modelstate.reference_frame = (std::string) "world";
			modelstate.pose = start_pose;
			modelstate.twist = start_twist;

			ros::ServiceClient client = srv_nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
			gazebo_msgs::SetModelState setmodelstate;
			setmodelstate.request.model_state = modelstate;
			client.call(setmodelstate);
			
			// publish clock server
			clk_pub_->publish(conv_time_to_topic_(get_prv_state()));

			// non-ROS sleep command
			usleep(get_ts()*1e6L);

			// reset message flag
			n_msg_ = false;
		}

		ros::spinOnce(); // handle call backs
	}
	
	return EXIT_FAILURE;
}

Usv16RosInterface::~Usv16RosInterface(void) 
{
	ros::shutdown();
}


/////////////////////
// Private Methods //
/////////////////////

void Usv16RosInterface::signal_int_handler_(int sig_no)
{
	// shutdown ros on signal interupt
	ros::shutdown();
}

void Usv16RosInterface::actuator_callback_(const usv16_msgs::Usv16ActuatorInputsConstPtr& msg)
{	
 	act_msg_ = msg; // store pointers
 	n_msg_ = true;  // set new new msg flag to true
 	fst_msg_ = true;
}

bool Usv16RosInterface::reset_service_callback_(std_srvs::Empty::Request& req, std_srvs::Empty::Request& res)
{
	ROS_INFO_STREAM(db_name << "Simulation state reset requested...");

	// reset to initial state
	reset_ini_state();
}

/**
 * @brief 	Populates the initial state of the EOM from the ROS parameter server.
 * @details Populates the initial state of the EOM from the ROS parameter server. If this is not called, the
 * 			initial conditions defaults to zero.
 */
void Usv16RosInterface::populate_ini_sim_(void)
{
	double ts;
	state_t ini_st; // initial state struct

	// vectors for middleway between looking up ROS params
	vector<double> v_pose, v_vel, v_acc;

	// no elegent way to do this... just straight conversion
	ros::param::get("sim/ini/pose", v_pose);
	ros::param::get("sim/ini/vel",	v_vel);
	ros::param::get("sim/ini/acc",	v_acc);
	ros::param::get("sim/ts",ts);

	// ROS Error stream debug messages
	if(v_pose.size() != 3)
		ROS_ERROR_STREAM(db_name << "Incorrect paramter size: " << "sim/ini/pose");

	if(v_vel.size() != 3)
		ROS_ERROR_STREAM(db_name << "Incorrect paramter size: " << "sim/ini/vel");

	if(v_acc.size() != 3)
		ROS_ERROR_STREAM(db_name << "Incorrect paramter size: " << "sim/ini/acc");

	// store in initialization structure
	ini_st.pose[0] = v_pose[0];
	ini_st.pose[1] = v_pose[1];
	ini_st.pose[2] = v_pose[2];

	ini_st.vel[0] = v_vel[0];
	ini_st.vel[1] = v_vel[1];
	ini_st.vel[2] = v_vel[2];

	ini_st.acc[0] = v_acc[0];
	ini_st.acc[1] = v_acc[1];
	ini_st.acc[2] = v_acc[2];

	// populate from inherited public functions
	set_ini_state(ini_st);
	set_ts(ts);
}

usv16_msgs::Usv16State Usv16RosInterface::conv_sim_to_topic_(const state_t& sim_st) const
{
	usv16_msgs::Usv16State state;

	state.t 			= sim_st.t;

	state.pose.x 		= sim_st.pose[0];
	state.pose.y 		= sim_st.pose[1];
	state.pose.theta 		= sim_st.pose[2];

	state.vel.linear.x = sim_st.vel[0];
	state.vel.linear.y = sim_st.vel[1];
	state.vel.linear.z = 0.0;

	state.vel.angular.x = 0.0;
	state.vel.angular.y = 0.0;
	state.vel.angular.z = sim_st.vel[2];

	return state;
}

rosgraph_msgs::Clock Usv16RosInterface::conv_time_to_topic_(const state_t& sim_st) const
{
	rosgraph_msgs::Clock t;

	// convert from floating point to time
	t.clock.sec = floor(sim_st.t);
	t.clock.nsec = (long) ((sim_st.t-floor(sim_st.t))*1e9L);
	
	return t;
}


Usv16Eom3Dof::Vec4f64_t  Usv16RosInterface::conv_topic_to_act_(const usv16_msgs::Usv16ActuatorInputsConstPtr& msg) const
{
	return Vec4f64_t({msg->actuator_inputs[0],msg->actuator_inputs[1],msg->actuator_inputs[2],msg->actuator_inputs[3]});
}

usv16_msgs::Usv16State Usv16RosInterface::simulate_topic_wrapper_(const usv16_msgs::Usv16ActuatorInputsConstPtr& msg)
{
	return conv_sim_to_topic_(sim_one_time_step(conv_topic_to_act_(msg)));
}

//This block of code will be how we convert the ship from earth fixed frame to world frame,
//as well as then converting that world frame info to a quaternion to be sent to gazebo.
//Note that this is truly a NED to EFF conversion, however the naming convention is such that
//the eff has been defined as NED and world is world.
