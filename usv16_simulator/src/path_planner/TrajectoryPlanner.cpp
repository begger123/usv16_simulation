#include "path_planner/TrajectoryPlanner.h"

#include <math.h>


TrajectoryPlanner::TrajectoryPlanner(ConfigFile& ioCfg):mCfg(ioCfg)
{
	init();
}

void TrajectoryPlanner::init()
{
	mPSpaceDimX = mCfg.read<double>("PSPACE_DIM_NORTH_X", 0);
	mPSpaceDimY = mCfg.read<double>("PSPACE_DIM_EAST_Y", 0);
	mPSpaceDimPsi = mCfg.read<double>("PSPACE_DIM_PSI", 0);
	mPSpaceDeltaX = mCfg.read<double>("PSPACE_DELTA_X", 0);
	mPSpaceDeltaY = mCfg.read<double>("PSPACE_DELTA_Y", 0);
	mPSpaceDeltaPsi = mCfg.read<double>("PSPACE_DELTA_PSI", 0);
	mAlpha = mCfg.read<double>("ALPHA", 0);
	mToleranceToGoal = mCfg.read<double>("GOAL_TOLERANCE", 0);
	mNumberOfActions = mCfg.read<double>("NUMBER_OF_ACTIONS", 0);
	mSmoothingWeight = mCfg.read<double>("WEIGHT_SMOOTH", 0);
	mSmoothingDelta = mCfg.read<double>("WEIGHT_DELTA", 0);
	mSmoothingTol = mCfg.read<double>("SMOOTHING_TOLERANCE", 0);
	mDistanceBetweenWP = mCfg.read<double>("DISTANCE_BETWEEN_WP", 0);
	mToleranceOfGoalPsi = mCfg.read<double>("GOAL_PSI_TOLERANCE",0);
	mMotionGoalDistance = mCfg.read<double>("MOTION_GOAL_SAFE_DISTANCE",0);
	mDummyObstacleDistance = mCfg.read<double>("DUMMY_OBSTACLE_DISTANCE",0);
	mFollowDistRing = mCfg.read<double>("MIN_FOLLOW_DISTANCE_RING",0);
	mSoftObstCost = mCfg.read<double>("SOFT_OBSTACLE_COST",0);
	mPControllerValue = mCfg.read<float>("P_VALUE_FOR_SPEED_CONTROLLER",0);
	mStopingDistance = mCfg.read<float>("MIN_FOLLOW_DISTANCE_RING",0);
	mMaxSpeedPublished = mCfg.read<float>("MAX_SPEED_OF_BOAT",0);

	initDataStructures();

	mDiscreteToleranceGoalPsi = Utils::round(mToleranceOfGoalPsi/mPSpaceDeltaPsi);
	/*
	mGScore.resize(boost::extents[mPSpaceDimX][mPSpaceDimY][mPSpaceDimPsi]);
	mHScore.resize(boost::extents[mPSpaceDimX][mPSpaceDimY][mPSpaceDimPsi]);
	mInClosedSet.resize(boost::extents[mPSpaceDimX][mPSpaceDimY][mPSpaceDimPsi]);
	mInOpenSet.resize(boost::extents[mPSpaceDimX][mPSpaceDimY][mPSpaceDimPsi]);
	mSelectedNodes.resize(boost::extents[mPSpaceDimX][mPSpaceDimY][mPSpaceDimPsi]);
	mActionOfSelectedNodes.resize(boost::extents[mPSpaceDimX][mPSpaceDimY][mPSpaceDimPsi]);
	mObstacleSet.resize(boost::extents[mPSpaceDimX][mPSpaceDimY]);

	mCurrentNeighbour.resize(3);
	mCurrentNeighbour.clear();
	mAllSelectedNodes.clear();
	mVerticesIDX.clear();*/
	mTrajectory.clear();

	mActionSet.clear(); mIntermediateActionSet.clear();
	mActionSet.resize(8); mIntermediateActionSet.resize(8);

	for(int k=0;k<mPSpaceDimPsi;k++)
		mActionSet[k].resize(mNumberOfActions);

	// mTrajVisualFile.open("/home/brual/Desktop/AStar3D_Path_Planner_09272016/Visualization/TrajectoryVisual.txt");
	mTrajVisualFile.open("TrajectoryVisual.txt");
	mObstOP.open("Obstacles.csv");
	mTrajectoryOP.open("Output_Trajectory.csv");

	Parser lParserObj;
//	mActionSet = lParserObj.parseControlActions("../config/ControlActions.txt");
    mActionSet = lParserObj.parseControlActions("/home/travematics/Documents/msl_clone_cws/src/usv16_simulation/usv16_simulator/config/ControlActions.txt");
//    mIntermediateActionSet = lParserObj.parseIntermediateControlActions("../config/IntermediateControlActions.txt");
    mIntermediateActionSet = lParserObj.parseIntermediateControlActions("/home/travematics/Documents/msl_clone_cws/src/usv16_simulation/usv16_simulator/config/IntermediateControlActions.txt");
	mParsedObstacles.clear();
	// mParsedObstacles = lParserObj.loadObstacles("../obstacles/ObstaclesAll.txt");
//    mObstaclesCenters = lParserObj.loadObstaclesCenter("../obstacles/ObstaclesCenter.txt");
    mObstaclesCenters = lParserObj.loadObstaclesCenter("/home/travematics/Documents/msl_clone_cws/src/usv16_simulation/usv16_simulator/config/obstacles/ObstaclesCenter.txt");
	mAllObstacles.clear();
	setStaticObstacles();
	
	// mAllObstacles = mObstaclesCenters;
	/*
	for(int i=0;i<mParsedObstacles.size();i++)
	{
		if(mParsedObstacles[i][0] > 0 && mParsedObstacles[i][1] > 0)
		{
			mObstacleSet[mParsedObstacles[i][0]][mParsedObstacles[i][1]]=1;
			mObstOP<<mParsedObstacles[i][0]<<","<<mParsedObstacles[i][1]<<endl;
			mObstVisualFile<<mParsedObstacles[i][0]<<","<<mParsedObstacles[i][1]<<";";
		}
	}*/
	cout<<"Fine!!"<<endl;
	//OPEN_SET.resize(1000000);
	//mVertexID=mCurrentNodeID=mCurrentNodeCost=mCurrentNeighbour_new_gscore=0;
}

void TrajectoryPlanner::setStaticObstacles()
{
	mAllObstacles.clear();
	mStaticObstacles.clear();
	for(int i =0;i<mObstaclesCenters.size();i++)
	{
		vector<double> lTemp;
		lTemp.push_back(mObstaclesCenters[i][0]);
		lTemp.push_back(mObstaclesCenters[i][1]);
		lTemp.push_back(mObstaclesCenters[i][2]);
		mAllObstacles.push_back(lTemp);
		mStaticObstacles.push_back(lTemp);
	}
}

vector<vector<double> >  TrajectoryPlanner::getStaticObstacles()
{
	return mStaticObstacles;
}

void TrajectoryPlanner::setDynamicObstacles(vector<vector<double> > inObst)
{
	for(int i =0;i<inObst.size();i++)
	{
		vector<double> lTemp;
		lTemp.push_back(inObst[i][0]);
		lTemp.push_back(inObst[i][1]);
		lTemp.push_back(8);
		mAllObstacles.push_back(lTemp);

		vector<double> lAdditionalObst;
		lAdditionalObst.clear();
		lAdditionalObst.push_back(inObst[i][0]+8*cos(inObst[i][2]));
		lAdditionalObst.push_back(inObst[i][1]+8*sin(inObst[i][2]));
		lAdditionalObst.push_back(8);
		mAllObstacles.push_back(lAdditionalObst);
	}
}

void TrajectoryPlanner::initDataStructures()
{
	//mGScore.
	array3D mGScore;
	array3D mHScore;
	array3D mInClosedSet;
	array3D mInOpenSet;
	array3D mSelectedNodes;
	//array3D mActionOfSelectedNodes;
	
	MaxHeap<double> OPEN_SET;
	mGScore.resize(boost::extents[mPSpaceDimX][mPSpaceDimY][mPSpaceDimPsi]);
	mHScore.resize(boost::extents[mPSpaceDimX][mPSpaceDimY][mPSpaceDimPsi]);
	mInClosedSet.resize(boost::extents[mPSpaceDimX][mPSpaceDimY][mPSpaceDimPsi]);
	mInOpenSet.resize(boost::extents[mPSpaceDimX][mPSpaceDimY][mPSpaceDimPsi]);
	mSelectedNodes.resize(boost::extents[mPSpaceDimX][mPSpaceDimY][mPSpaceDimPsi]);
	mActionOfSelectedNodes.resize(boost::extents[mPSpaceDimX][mPSpaceDimY][mPSpaceDimPsi]);
	mObstacleSet.resize(boost::extents[mPSpaceDimX][mPSpaceDimY]);

	mCurrentNeighbour.clear();
	mCurrentNeighbour.resize(3);
	mAllSelectedNodes.clear();
	mVerticesIDX.clear();
	mStaticObstacles.clear();
	//mTrajectory.clear();

	//OPEN_SET.
	OPEN_SET.resize(1000000);
	mVertexID=mCurrentNodeID=mCurrentNodeCost=mCurrentNeighbour_new_gscore=0;
}
TrajectoryPlanner::~TrajectoryPlanner(void)
{}

void TrajectoryPlanner::printTest()
{
	for(int i=0;i<mIntermediateActionSet.size();i++)
	{
		vector<Action> lOneAction = mIntermediateActionSet[i];
		for(int j=0;j<lOneAction.size();j++)
		{
			Action lSingleAction = lOneAction[j];
			cout<<i+1<<" mIntermediateActionSet: "<<j+1<<": "<<lSingleAction.getActionProfile()[0]<<","<<lSingleAction.getActionProfile()[1]<<
				","<<lSingleAction.getActionProfile()[2]<<";"<<lSingleAction.getActionCost()<<endl;
		}
	}
	for(int i=0;i<mActionSet.size();i++)
	{
		vector<Action> lOneAction = mActionSet[i];
		for(int j=0;j<lOneAction.size();j++)
		{
			Action lSingleAction = lOneAction[j];
			cout<<i+1<<" mActionSet: "<<j+1<<": "<<lSingleAction.getActionProfile()[0]<<","<<lSingleAction.getActionProfile()[1]<<
				","<<lSingleAction.getActionProfile()[2]<<";"<<lSingleAction.getActionCost()<<endl;
		}
	}
	
}

vector<double> TrajectoryPlanner::computeTrajectory(vector<double> inUSV12Pose, vector<double> inUSV12Vel, vector<double> inGUSSPose)
{
		//mGScore.
	array3D mGScore;
	array3D mHScore;
	array3D mInClosedSet;
	array3D mInOpenSet;
	array3D mSelectedNodes;
	//array3D mActionOfSelectedNodes;
	
	MaxHeap<double> OPEN_SET;
	mGScore.resize(boost::extents[mPSpaceDimX][mPSpaceDimY][mPSpaceDimPsi]);
	mHScore.resize(boost::extents[mPSpaceDimX][mPSpaceDimY][mPSpaceDimPsi]);
	mInClosedSet.resize(boost::extents[mPSpaceDimX][mPSpaceDimY][mPSpaceDimPsi]);
	mInOpenSet.resize(boost::extents[mPSpaceDimX][mPSpaceDimY][mPSpaceDimPsi]);
	mSelectedNodes.resize(boost::extents[mPSpaceDimX][mPSpaceDimY][mPSpaceDimPsi]);
	mActionOfSelectedNodes.resize(boost::extents[mPSpaceDimX][mPSpaceDimY][mPSpaceDimPsi]);
	mObstacleSet.resize(boost::extents[mPSpaceDimX][mPSpaceDimY]);

	mCurrentNeighbour.clear();
	mCurrentNeighbour.resize(3);
	mAllSelectedNodes.clear();
	mVerticesIDX.clear();
	//mTrajectory.clear();

	//OPEN_SET.
	OPEN_SET.resize(1000000);
	mVertexID=mCurrentNodeID=mCurrentNodeCost=mCurrentNeighbour_new_gscore=0;



	if(inUSV12Pose.size()<1 || inUSV12Vel.size()<1 || inGUSSPose.size()<1)
	{
		cout<<"No Data to find path...!!!"<<endl;
		return mTrajectory;
	}
	//CONVERSION OF INPUTS
	vector<double>lUSV12Pose,lGUSSPose;
	for(int i=0;i<2;i++)
	{
		lUSV12Pose.push_back(Utils::round(inUSV12Pose[i]));
		lGUSSPose.push_back(Utils::round(inGUSSPose[i]));
	}
	lUSV12Pose.push_back(Utils::round(inUSV12Pose[2]/mPSpaceDeltaPsi) + 1);
	lGUSSPose.push_back(Utils::round(inGUSSPose[2]/mPSpaceDeltaPsi) + 1);

	//cout<<"USV12 IP: "<<lUSV12Pose[0]<<":"<<lUSV12Pose[1]<<":"<<lUSV12Pose[2]<<endl;
	//cout<<"GUSS IP: "<<lGUSSPose[0]<<":"<<lGUSSPose[1]<<":"<<lGUSSPose[2]<<endl;

	if(lUSV12Pose[2]>8)
	{
		lUSV12Pose[2]=1;
	}
	if(lGUSSPose[2]>8)
	{
		lGUSSPose[2]=1;
	}

	//INPUT PARAMETERS VERIFICATION
	if(lUSV12Pose[0]>=mPSpaceDimX || lUSV12Pose[1]>=mPSpaceDimY ||lUSV12Pose[2]>9||
				lUSV12Pose[0]<0 || lUSV12Pose[1]<0 || lUSV12Pose[2]<0)
	{
		cout<<"Enter USV12 location in proper bounds...!!!"<<endl;
		cout<<"USV12: "<<lUSV12Pose[0]<<" "<<lUSV12Pose[1]<<" "<<lUSV12Pose[2]<<endl;
		return mTrajectory;
	}
	
	if(lGUSSPose[0]>=mPSpaceDimX || lGUSSPose[1]>=mPSpaceDimY ||lGUSSPose[2]>9||
				lGUSSPose[0]<0 || lGUSSPose[1]<0 || lGUSSPose[2]<0)
	{
		cout<<"Enter GUSS location in proper bounds...!!!"<<endl;
		cout<<"GUSS: "<<lGUSSPose[0]<<" "<<lGUSSPose[1]<<" "<<lGUSSPose[2]<<endl;
		return mTrajectory;
	}

	//Final Orientation tolerance
	int lLowerOri, lUpperOri;
	if(lGUSSPose[2]==1)
	{
		lUpperOri=8-mDiscreteToleranceGoalPsi+1;
		lLowerOri=2+mDiscreteToleranceGoalPsi-1;
	}
	else if(lGUSSPose[2]==8)
	{
		lUpperOri=7-mDiscreteToleranceGoalPsi+1;
		lLowerOri=1+mDiscreteToleranceGoalPsi-1;
	}
	else
	{
		lLowerOri=lGUSSPose[2]-mDiscreteToleranceGoalPsi;//mDiscreteToleranceGoalPsi;
		lUpperOri=lGUSSPose[2]+mDiscreteToleranceGoalPsi;//mDiscreteToleranceGoalPsi;
	}
	
	//INITIALIZATION OF COST PARAMETERS
	mHScore[lUSV12Pose[0]][lUSV12Pose[1]][lUSV12Pose[2]]=Utils::norm(lUSV12Pose[0],lUSV12Pose[1],lGUSSPose[0],lGUSSPose[1]);
	mInOpenSet[lUSV12Pose[0]][lUSV12Pose[1]][lUSV12Pose[2]]=1;
	mVerticesIDX.push_back(lUSV12Pose);
	OPEN_SET.push(0,mVertexID);//OPEN_SET(cost,vertex);
	int count = 0;

	bool isUSVCollision =false;
	isUSVCollision = boatSoftColDetection(lUSV12Pose);
	bool isTargetCollision = false;
	isTargetCollision = boatSoftColDetection(lGUSSPose);

	if(isUSVCollision)
		std::cout<<"USV Collision..!!"<<std::endl;

	if(isTargetCollision)
		std::cout<<"Target Collision..!!"<<std::endl;

	while(!OPEN_SET.empty())
	{
		mCurrentNodeCost=OPEN_SET.top().first;
		mCurrentNodeID=OPEN_SET.top().second;
		OPEN_SET.pop();//POP Mandatory to remove from priority queue
		mCurrentNode=mVerticesIDX[mCurrentNodeID];
		mInOpenSet[mCurrentNode[0]][mCurrentNode[1]][mCurrentNode[2]]=0;
		mInClosedSet[mCurrentNode[0]][mCurrentNode[1]][mCurrentNode[2]]=1;
		count = count +1;

		//Reconstruct Trajectory
		double lDistanceToGoal = Utils::norm(mCurrentNode[0],mCurrentNode[1],lGUSSPose[0],lGUSSPose[1]);

		if(lDistanceToGoal<mToleranceToGoal && lGUSSPose[2]!=1 && lGUSSPose[2]!=8 && (mCurrentNode[2]<=lUpperOri) && (mCurrentNode[2]>=lLowerOri) ||
				(lDistanceToGoal<mToleranceToGoal && lGUSSPose[2]==8 && (((mCurrentNode[2]<=8) && (mCurrentNode[2]>=lUpperOri))
					|| ((mCurrentNode[2]>=1) && (mCurrentNode[2]<=lLowerOri)))) ||
				(lDistanceToGoal<mToleranceToGoal && lGUSSPose[2]==1 && (((mCurrentNode[2]<=8) && (mCurrentNode[2]>=lUpperOri))
					|| ((mCurrentNode[2]>=1) && (mCurrentNode[2]<=lLowerOri)))))
	//	if(lDistanceToGoal<mToleranceToGoal && mCurrentNode[2]==lGUSSPose[2])
		{
			cout<<"Reconstructing Traj...!!!"<<endl;
			mAllSelectedNodes.clear();
			double lCurrID = mCurrentNodeID;
			vector<double> lTempNode,lCurrNode;
			lTempNode.clear();lCurrNode.clear();
			for(int x=0;x<3;x++)
			{
				lTempNode.push_back(mCurrentNode[x]);
				lCurrNode.push_back(mCurrentNode[x]);
			}
			
			double distance = 5;
			

			//speed controller
			//double speedForWP = inUSV12Vel[0];
			double lDistToGoal = Utils::norm(lUSV12Pose[0],lUSV12Pose[1],lGUSSPose[0],lGUSSPose[1]);
			

			//if(lDistToGoal>mFollowDistRing)
			float difference = (lDistToGoal-mFollowDistRing);
			float lSpeedGain = (difference*mPControllerValue);
			float speedForWP = inUSV12Vel[0]+lSpeedGain;
			
			if(lDistToGoal<mStopingDistance)
			{
				speedForWP=0;
			}

			if(speedForWP<0)
			{
				speedForWP = 0;
			}


			if(speedForWP>mMaxSpeedPublished)
			{
				speedForWP = mMaxSpeedPublished;
			}
			// cout<<"SPEED CONTROL PARAMETERS..!!!"<<endl;
			// cout<<"DIST TO GOAL: "<<lDistToGoal<<" Current Speed: "<<inUSV12Vel[0]<<" Speed Gain: "<<lSpeedGain<<" New Speed: "<<speedForWP<<endl;
			// cout<<"P Value: "<<mPControllerValue<<" Follow Ring Dist: "<<mFollowDistRing<<endl;

			lTempNode.push_back(speedForWP);
			mAllSelectedNodes.push_back(lTempNode);

			while(distance>1)
			{
				
				lCurrID = mSelectedNodes[lCurrNode[0]][lCurrNode[1]][lCurrNode[2]];
				lCurrNode.clear();
				lCurrNode.push_back(mVerticesIDX[lCurrID][0]);
				lCurrNode.push_back(mVerticesIDX[lCurrID][1]);
				lCurrNode.push_back(mVerticesIDX[lCurrID][2]);
				lCurrNode.push_back(speedForWP);				
				mAllSelectedNodes.push_back(lCurrNode);
				
				distance=Utils::norm(lUSV12Pose[0],lUSV12Pose[1],lCurrNode[0],lCurrNode[1]);

			}

			vector<vector<double> > lSmoothTrajectory = smoothTrajectory(mAllSelectedNodes);
			vector<vector<double> > lWaypointsOP;
			lWaypointsOP.clear();
			mTrajectory.clear();
			for(int a=mAllSelectedNodes.size()-1;a>-1;a--)
			{
				vector<double> wp=mAllSelectedNodes[a];
				vector<double> wp1 = lSmoothTrajectory[a];
				cout<<"WP: "<<wp1[0]<<","<<wp1[1]<<","<<wp1[3]<<endl;
				mTrajectoryOP<<wp[0]<<","<<wp[1]<<","<<wp[2]<<","<<wp[1]<<","<<wp[0]<<","<<wp1[1]<<","<<wp1[0]<<endl;
				lWaypointsOP.push_back(wp1);
				mTrajectory.push_back(wp1[0]);
				mTrajectory.push_back(wp1[1]);
				mTrajectory.push_back(wp1[3]);
			}
			for(int b=0;b<lWaypointsOP.size();b++)
			{
				mTrajVisualFile<<lWaypointsOP[b][0]<<","<<lWaypointsOP[b][1]<<","<<lWaypointsOP[b][2]<<";";
			}
			mTrajVisualFile<<"#"<<endl;
			return mTrajectory;
		}

		//Evaluate Neighbours
		vector<Action> lCurrentActionSet = mActionSet[mCurrentNode[2]];
		for(int i=0;i<lCurrentActionSet.size();i++)
		{
			Action lCurrentAction = lCurrentActionSet[i];

			mCurrentNeighbour.clear();
			mCurrentNeighbour.push_back(Utils::round(mCurrentNode[0] + (mDistanceBetweenWP/7)*lCurrentAction.getActionProfile()[0]));
			mCurrentNeighbour.push_back(Utils::round(mCurrentNode[1] + (mDistanceBetweenWP/7)*lCurrentAction.getActionProfile()[1]));
			mCurrentNeighbour.push_back(lCurrentAction.getActionProfile()[2]);//PSI of Action

			if(mCurrentNeighbour[0]>=mPSpaceDimX || mCurrentNeighbour[1]>=mPSpaceDimY ||mCurrentNeighbour[2]>mPSpaceDimPsi+1||
				mCurrentNeighbour[0]<2 || mCurrentNeighbour[1]<2)
			{	//Check in Closed Set
				continue;
			}

			if(mInClosedSet[mCurrentNeighbour[0]][mCurrentNeighbour[1]][mCurrentNeighbour[2]]==1)
			{
				continue;
			}

			
			double lSoftCollisionCost = 0;
			if(!isUSVCollision && !isTargetCollision)
			{
				if(colCheckVersion1())
				{//Check for collision with Obstacles	
					continue;
				}
			}
			else
			{
				//Soft Obstacle Collistion Cost
				if(colCheckVersion2())
				{//Check for collision with Obstacles
					continue;
				}
				if(softColDetection())
				{
					lSoftCollisionCost = mSoftObstCost;
				}
			}
			//If its not in OPEN & CLOSED set add in IDX vector
			if(mInOpenSet[mCurrentNeighbour[0]][mCurrentNeighbour[1]][mCurrentNeighbour[2]]!=1 && 
				mInClosedSet[mCurrentNeighbour[0]][mCurrentNeighbour[1]][mCurrentNeighbour[2]]!=1)
			{
				mVertexID++;
				mVerticesIDX.push_back(mCurrentNeighbour);
			}
			
			double lCurrentNode_gscore = mGScore[mCurrentNode[0]][mCurrentNode[1]][mCurrentNode[2]];
			mCurrentNeighbour_new_gscore = lCurrentNode_gscore+(mDistanceBetweenWP/7)*lCurrentAction.getActionCost();//Node gscore + action cost
			
			bool new_better = false;
			if(mInOpenSet[mCurrentNeighbour[0]][mCurrentNeighbour[1]][mCurrentNeighbour[2]]!=1)
			{
				mInOpenSet[mCurrentNeighbour[0]][mCurrentNeighbour[1]][mCurrentNeighbour[2]]=1;
				new_better=true;
			}
			else if(mCurrentNeighbour_new_gscore<mGScore[mCurrentNeighbour[0]][mCurrentNeighbour[1]][mCurrentNeighbour[2]])
				new_better=true;

			if(new_better)
			{
				mSelectedNodes[mCurrentNeighbour[0]][mCurrentNeighbour[1]][mCurrentNeighbour[2]]=mCurrentNodeID;
				mActionOfSelectedNodes[mCurrentNeighbour[0]][mCurrentNeighbour[1]][mCurrentNeighbour[2]]=i;
				mGScore[mCurrentNeighbour[0]][mCurrentNeighbour[1]][mCurrentNeighbour[2]]=mCurrentNeighbour_new_gscore;
				mHScore[mCurrentNeighbour[0]][mCurrentNeighbour[1]][mCurrentNeighbour[2]]=
						Utils::norm(mCurrentNeighbour[0],mCurrentNeighbour[1],lGUSSPose[0],lGUSSPose[1]);
				double cost = -(mGScore[mCurrentNeighbour[0]][mCurrentNeighbour[1]][mCurrentNeighbour[2]]+
									mAlpha*mHScore[mCurrentNeighbour[0]][mCurrentNeighbour[1]][mCurrentNeighbour[2]]+lSoftCollisionCost);
				OPEN_SET.push(cost,mVertexID);
			}

		}

	}
	return mTrajectory;
}

bool TrajectoryPlanner::colCheck()
{
	bool isCollision = false;
	vector<double> lDirVec,lUnitDirVec;
	lDirVec.resize(2);
	lUnitDirVec.resize(2);
	lDirVec[0]=mCurrentNode[0]-mCurrentNeighbour[0];
	lDirVec[1]=mCurrentNode[1]-mCurrentNeighbour[1];
	
	double lDirVecMag = Utils::norm(lDirVec[0],lDirVec[1],0,0);
	lUnitDirVec[0]=lDirVec[0]/lDirVecMag;
	lUnitDirVec[1]=lDirVec[1]/lDirVecMag;
	double lDirSeg=0;
	while(lDirSeg<lDirVecMag)
	{
		vector<double> lDirNode,lDirNodeCeil;
		lDirNode.clear();lDirNodeCeil.clear();
		lDirNode.resize(2);lDirNodeCeil.resize(2);
		
		lDirNode[0] = Utils::round((mCurrentNode[0]+lDirSeg*lUnitDirVec[0])/mPSpaceDeltaX);
		lDirNode[1] = Utils::round((mCurrentNode[1]+lDirSeg*lUnitDirVec[1])/mPSpaceDeltaY);

		lDirNodeCeil[0] = ceil((mCurrentNode[0]+lDirSeg*lUnitDirVec[0])/mPSpaceDeltaX);
		lDirNodeCeil[1] = ceil((mCurrentNode[1]+lDirSeg*lUnitDirVec[1])/mPSpaceDeltaY);
		
		
		if(lDirNode[0]<mPSpaceDimX && lDirNode[1]<mPSpaceDimY && lDirNode[0]>0 && lDirNode[1]>0 &&
			lDirNodeCeil[0]<mPSpaceDimX && lDirNodeCeil[1]<mPSpaceDimY && lDirNodeCeil[0]>0 && lDirNodeCeil[1]>0) 
			{
				if(mObstacleSet[lDirNode[0]][lDirNode[1]]>0)
				{
					return true;
				}
				if(mObstacleSet[lDirNodeCeil[0]][lDirNodeCeil[1]]>0)
				{
					return true;
				}
			}
		lDirSeg++;
	}
	return isCollision;
}

bool TrajectoryPlanner::colCheckVersion1()
{
	for(int i=0;i<mAllObstacles.size();i++)
	{
		double lDistanceFromObst = Utils::norm(mCurrentNeighbour[0],mCurrentNeighbour[1],
													mAllObstacles[i][0],mAllObstacles[i][1]);
		if(lDistanceFromObst<(mAllObstacles[i][2]+5))
		{
			return true;
		}
	}
	return false;
}


bool TrajectoryPlanner::colCheckVersion2()
{
	for(int i=0;i<mAllObstacles.size();i++)
	{
		double lDistanceFromObst = Utils::norm(mCurrentNeighbour[0],mCurrentNeighbour[1],
													mAllObstacles[i][0],mAllObstacles[i][1]);
		if(lDistanceFromObst<(mAllObstacles[i][2]+5-mDummyObstacleDistance))
		{
			return true;
		}
	}
	return false;
}

bool TrajectoryPlanner::softColDetection()
{
	bool flag = false;
	for(int i=0;i<mAllObstacles.size();i++)
	{
		double lDistanceFromObst = Utils::norm(mCurrentNeighbour[0],mCurrentNeighbour[1],
													mAllObstacles[i][0],mAllObstacles[i][1]);
		if(((mAllObstacles[i][2]- mDummyObstacleDistance)<lDistanceFromObst) && (lDistanceFromObst<(mAllObstacles[i][2]+5)))
		{
			flag =  true;
		}
	}
	return flag;
}

bool TrajectoryPlanner::boatSoftColDetection(vector<double> inState)
{
	bool flag = false;
	for(int i=0;i<mAllObstacles.size();i++)
	{
		double lDistanceFromObst = Utils::norm(inState[0],inState[1],
													mAllObstacles[i][0],mAllObstacles[i][1]);
		if(((mAllObstacles[i][2]- mDummyObstacleDistance) < lDistanceFromObst) && (lDistanceFromObst < (mAllObstacles[i][2]+5)))
		{
				flag =  true;
		}
	}

	return flag;
}

vector<vector<double> > TrajectoryPlanner::smoothTrajectory(vector<vector<double> > inTrajectory)
{
	vector<vector<double> > lSmoothTraj = inTrajectory;
	double lDelta = mSmoothingTol;
	while(lDelta>= mSmoothingTol)
	{
		lDelta=0;
		for(int j=1;j<inTrajectory.size()-1;j++)
		{
			vector<double> vertex_old = lSmoothTraj[j];
			lSmoothTraj[j][0]= lSmoothTraj[j][0]+mSmoothingDelta*(inTrajectory[j][0]-lSmoothTraj[j][0])+
				mSmoothingWeight*(lSmoothTraj[j+1][0]+lSmoothTraj[j-1][0]-2*lSmoothTraj[j][0]);
			lSmoothTraj[j][1]= lSmoothTraj[j][1]+mSmoothingDelta*(inTrajectory[j][1]-lSmoothTraj[j][1])+
				mSmoothingWeight*(lSmoothTraj[j+1][1]+lSmoothTraj[j-1][1]-2*lSmoothTraj[j][1]);

			lDelta = lDelta + Utils::norm(vertex_old[0],vertex_old[1],lSmoothTraj[j][0],lSmoothTraj[j][1]);
		}
	}

	return lSmoothTraj;

}
