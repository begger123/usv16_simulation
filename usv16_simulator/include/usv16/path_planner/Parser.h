#pragma once
#include "ConfigFile.h"
#include "Action.h"
#include <cstdlib>
#include <vector>

using namespace std;
class Parser
{
public:
	Parser(void);
	~Parser(void);

	vector<vector<double> > parseObstaclesCont(std::string& inLine);
	vector<vector<double> > parseObstacles(const char* inLine);
	vector<vector<double> > loadObstacles(std::string inFilename);
	//vector<vector<Action> > Parser::parseControlActions(std::string& inLine);
	vector<vector<Action> > parseControlActions(const char* inLine);
	vector<vector<Action> > parseIntermediateControlActions(const char* inLine);
	vector<vector<double> > loadObstaclesCenter(std::string inFilename);
	vector<vector<double> > parseObstaclesCenter(std::string& inLine);
};

