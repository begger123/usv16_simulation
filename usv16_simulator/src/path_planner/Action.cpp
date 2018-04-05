#include "path_planner/Action.h"


Action::Action(void)
{
	mProfile.clear();
	mProfile.resize(3);
	mCost=0;
}

Action::~Action(void)
{
}

Action::Action(vector<double> inProfile, double inCost)
{
	mProfile.clear();
	mProfile.resize(3);
	for(int i=0;i<3;i++)
		mProfile[i]=inProfile[i];

	mCost=inCost;
}

Action::Action(const Action &inAction)
{
	mProfile.clear();
	mProfile.resize(3);
	mProfile=inAction.mProfile;
	mCost=inAction.mCost;
}

vector<double> Action::getActionProfile()
{
	return mProfile;
}

double Action::getActionCost()
{
	return mCost;
}
