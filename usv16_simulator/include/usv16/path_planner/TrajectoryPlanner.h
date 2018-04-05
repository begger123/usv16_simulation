#pragma once
#include <vector>
#include "Utils.h"
#include "ConfigFile.h"
#include <path_planner/multi_array.hpp>
#include <fstream>
#include "MyHeap.h"
#include "Action.h"
#include "Parser.h"

typedef boost::multi_array<int,3> array3D;
typedef boost::multi_array<int,2> array2D;
typedef array3D::index index3D;
//array3D::extent_gen extents;

using namespace std;
class TrajectoryPlanner
{
private:
	ConfigFile mCfg;
	double mPSpaceDimX,mPSpaceDimY,mPSpaceDimPsi;
	double mPSpaceDeltaX, mPSpaceDeltaY, mPSpaceDeltaPsi;
	double mNumberOfActions,mAlpha, mDistanceBetweenWP, mToleranceOfGoalPsi,mDiscreteToleranceGoalPsi;
	double mVertexID,mCurrentNodeCost,mCurrentNodeID,mCurrentNeighbour_new_gscore;
	int mToleranceToGoal;
	double mMotionGoalDistance, mDummyObstacleDistance;
	vector<double> mCurrentNode,mCurrentNeighbour;
	vector<vector<double> > mVerticesIDX,mObstacles,mAllSelectedNodes,mParsedObstacles,mObstaclesCenters,mAllObstacles;
	vector<vector<double> > mStaticObstacles;
	//MaxHeap<double> OPEN_SET;
	vector<vector<Action> > mActionSet,mIntermediateActionSet;
	ofstream mTrajectoryOP,mObstOP,mTrajVisualFile, mObstVisualFile;
	double mSmoothingWeight,mSmoothingDelta,mSmoothingTol;
	double mFollowDistRing,mSoftObstCost;
	float mPControllerValue,mMaxSpeedPublished,mStopingDistance;
	/*
	array3D mGScore;
	array3D mHScore;
	array3D mInClosedSet;
	array3D mInOpenSet;
	array3D mSelectedNodes;*/
	array3D mActionOfSelectedNodes;
	array2D mObstacleSet;
	void init();
	bool colCheck();
	bool colCheckVersion1();
	bool colCheckVersion2();
	bool boatSoftColDetection(vector<double> inState);
	bool softColDetection();
	vector<vector<double> > smoothTrajectory(vector<vector<double> > inTrajectory);
	

public:
	TrajectoryPlanner(ConfigFile& ioCfg);
	~TrajectoryPlanner(void);
	vector<double> mTrajectory;
	void printTest();
	vector<double> computeTrajectory(vector<double> lUSV12Pose, vector<double> inUSV12Vel, vector<double> lGUSSPose);
	void initDataStructures();
	void setStaticObstacles();
	void setDynamicObstacles(vector<vector<double> > inObst);
	vector<vector<double> > getStaticObstacles();
};

