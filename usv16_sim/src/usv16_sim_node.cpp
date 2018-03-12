/**
 * @file usv16_ros_node.cpp
 * @author Ivan Bertaska
 * @date 29 Jun 2016
 * @brief File containing example of doxygen usage for quick reference.
 *
 * Here typically goes a more extensive explanation of what the header
 * defines. Doxygens tags are words preceeded by either a backslash @\
 * or by an at symbol @@.
 */


#include <iostream>
#include <cmath>

#include "ros/ros.h"
#include "usv16_sim/usv16_ros_interface.h"
#include "usv16_sim/usv16_dyn.h" 				// <- Dynamic equations of motion (from Matlab Coder)

 void my_actuators_function(const double * u, double * tau)
 {
 	const double lx = 2.0; // Longitudinal component from CG 
 	const double ly = 1.0; // Lateral component from CG
	
	// sanity checks
	if(NULL == tau || NULL == u)
		ROS_ERROR_STREAM("Actuator function input is null.");	
 
	// perform calculations to determine force on system
	const double X = u[0]*cos(u[1]) + u[2]*cos(u[3]);
	const double Y = u[0]*sin(u[1]) + u[2]*sin(u[3]);
	const double N = ly * (u[0]*cos(u[1]) - u[2]*cos(u[3])) 
					-lx * (u[0]*sin(u[1]) + u[2]*sin(u[3]));

	// store in tau value
	tau[0] = X;
	tau[1] = Y;
	tau[2] = N;

	return;
 }

int main(int argc, char** argv)
{
	// Initialize ROS interface with 3dof simulation of the USV16
	Usv16RosInterface usv16_ros_interface(argc,argv);

	// initialize matlab libraries
	usv16_dyn_initialize();

	// setup actuator and dynamics functions
	usv16_ros_interface.set_actuators_function(&my_actuators_function);
	
	/**
	 *	The "usv16_dyn" function is part of the "ml_usv16_dyn" library
	 *	whose SOURCE was generated through Matlab Coder. It is build within
	 *	the catkin environment and linked at run-time (built as shared object).	
	 */
	usv16_ros_interface.set_dynamics_function(&usv16_dyn); 

	// run simulation.
	return usv16_ros_interface.simulate();
}