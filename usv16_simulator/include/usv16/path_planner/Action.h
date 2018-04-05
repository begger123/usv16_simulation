#pragma once
#include <vector>
using namespace std;

class Action
{
private:
	vector<double> mProfile;
	double mCost;
public:
	Action(void);
	~Action(void);
	Action(vector<double> inProfile, double inCost);
	Action(const Action &inAction);
	vector<double> getActionProfile();
	double getActionCost();
	
};

