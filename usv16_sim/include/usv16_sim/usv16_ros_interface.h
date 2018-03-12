#ifndef USV16_ROS_INTERFACE_HEADER_INCLUDED
#define USV16_ROS_INTERFACE_HEADER_INCLUDED

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


#include <memory> // <- auto_ptrs since I have no idea how ros handles exceptions

#include "std_srvs/Empty.h"
#include "rosgraph_msgs/Clock.h" // <- clock server

#include "usv16_msgs/Usv16ActuatorInputs.h"
#include "usv16_msgs/Usv16State.h"
#include "usv16_sim/usv16_eom_3dof.h"

// forward declaraton
namespace ros {
	class NodeHandle;
	class Subscriber;
	class Publisher;
	class ServiceServer;
}

class Usv16RosInterface : public Usv16Eom3Dof
{
public:

	/////////////////
	// Constructor //
	/////////////////

	// consturctor for class
	Usv16RosInterface(int argc, char** argv);

	////////////////////
	// Public Methods //
	////////////////////
	
	// simulate
	int simulate(void);

	////////////////
	// Destructor //
	////////////////

	~Usv16RosInterface(void);

private:

	/////////////////////
	// Private Methods //
	/////////////////////

	/**
	 * @brief 	Wrapper to convert the ROS topics to dynamic equations of motion and back.
	 * @details Wrapper to convert the ROS topics to dynamic equations of motion and back.
	 * 
	 * @param msg	"actuator" topic message  
	 * @return 		State of the vehicle after 1 timestep
	 */
	usv16_msgs::Usv16State simulate_topic_wrapper_(const usv16_msgs::Usv16ActuatorInputsConstPtr& msg);
	
	static void signal_int_handler_(int sig_no);
	void actuator_callback_(const usv16_msgs::Usv16ActuatorInputsConstPtr& msg);
	bool reset_service_callback_(std_srvs::Empty::Request& req,std_srvs::Empty::Request& res);


	void populate_ini_sim_(void);
	usv16_msgs::Usv16State conv_sim_to_topic_(const state_t& sim_st) const;
	rosgraph_msgs::Clock conv_time_to_topic_(const state_t& sim_st) const;
	Vec4f64_t conv_topic_to_act_(const usv16_msgs::Usv16ActuatorInputsConstPtr& msg) const;

	////////////////////
	// Private Fields //
	////////////////////

	// ROS specific nodes. Wrapped in unique_ptr since I have no idea how 
	// ROS handles exceptions.
	std::unique_ptr<ros::NodeHandle> act_nh_;	// actuator node handle	
	std::unique_ptr<ros::NodeHandle> st_nh_;	// state node handle
	std::unique_ptr<ros::Subscriber> sub_;		// subscriber handle
	
	std::unique_ptr<ros::Publisher> st_pub_;		// state publisher hnadle
	std::unique_ptr<ros::Publisher> clk_pub_;		// clock publisher handle

	// pointer to handle message data
	usv16_msgs::Usv16ActuatorInputsConstPtr act_msg_; // actuator message pointer

};

#endif