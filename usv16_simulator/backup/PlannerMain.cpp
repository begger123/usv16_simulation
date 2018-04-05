#include <iostream>
#include "path_planner/ConfigFile.h"
#include "path_planner/TrajectoryPlanner.h"
#include "path_planner/Parser.h"
#include "path_planner/Action.h"
#include <time.h>

using namespace std;

int main(int argc, char *argv[]) 
{
	
	//COMMENT THE ABOVE TWO LINES AND UNCOMMENT THE BELOW ONE TO
	//..TEST THE PLANNER.
	
//    ConfigFile lCfg("../config/Config.txt");
    ConfigFile lCfg("~/Dev/USV/src/usv16_simulator/config/Config.txt");

	TrajectoryPlanner lPlanner(lCfg);
	vector<double>lUSV12, lGUSS, lUSV12Vel;
	lUSV12.push_back(5); //USV12 x co-ord (m)
	lUSV12.push_back(5); //USV12 x co-ord (m)
	lUSV12.push_back(0.785); //USV12 psi in radians

	lUSV12Vel.push_back(5); //USV12 surge speed
	lUSV12Vel.push_back(5); //USV12 sway speed

	lGUSS.push_back(50); //GUSS/GOAL x co-ord (m)
	lGUSS.push_back(260); //GUSS/GOAL y co-ord (m)
	lGUSS.push_back(0); //GUSS/GOAL psi in radians

	
	vector<double> lTraj = lPlanner.computeTrajectory(lUSV12,lUSV12Vel,lGUSS);
	if(lTraj.size()<1)
	{
		cout<<"Could not find trajectory...!!!"<<endl;
	}
	
	return 0;
}
