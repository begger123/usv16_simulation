#ifndef USV16_VIZ_HEADER_INCLUDED
#define USV16_VIZ_HEADER_INCLUDED

#include <memory>
#include <visualization_msgs/Marker.h>
#include "usv16_msgs/Usv16State.h"

//////////////////////////
// Forward Declarations //
//////////////////////////

namespace ros {

	class NodeHandle;
	class Subscriber;
	class Publisher;
}

namespace tf {

	class TransformBroadcaster;
}

class Usv16Viz
{
public:

	/////////////////
	// Constructor //
	/////////////////

	Usv16Viz(int argc, char **argv);
	
	////////////////////
	// Public Methods //
	////////////////////

	int run(void);

	////////////////
	// Destructor //
	////////////////

	~Usv16Viz();

private:

	////////////////////
	// Private Fields //
	////////////////////

	// Ros Nodes and publishers/subscribers
	std::unique_ptr<ros::NodeHandle> 	nh_;
	std::unique_ptr<ros::Subscriber>	st_sub_;		// state subscription
	// std::unique_ptr<ros::Subscriber>	ms_sub_;		// mission subscription
	std::unique_ptr<ros::Publisher>		traj_pub_;		// trajectory publication
	// std::unique_ptr<ros::Publisher>		wp_pub_;	// waypoint publication

	// ros visualization tools
	visualization_msgs::Marker trajectory_;

	// coordinate system transformations
	std::unique_ptr<tf::TransformBroadcaster> br_ef_;	// earth-fixed transformation broadcaster
	std::unique_ptr<tf::TransformBroadcaster> br_bf_;	// body-fixed transformation broadcaster

	usv16_msgs::Usv16StateConstPtr st_msg_; 				// state message pointer

	/////////////////////
	// Private Methods //
	/////////////////////

	void state_callback_(const usv16_msgs::Usv16StateConstPtr& msg);
	
	void broadcast_transforms_(void);
	void broadcast_transforms_(const usv16_msgs::Usv16StateConstPtr& msg);
	void update_trajectory_marker_(const usv16_msgs::Usv16StateConstPtr& msg);
	
	void initialize_trajectory_marker_(void);
	void initialize_waypoint_marker_(void);

};

#endif