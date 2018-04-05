#include "path_planner/Parser.h"
#include "path_planner/Tokenizer.h"



Parser::Parser(void)
{
}

Parser::~Parser(void)
{
}

vector<vector<double> > Parser::parseObstaclesCont(std::string& inLine)
{
	vector<vector<double> > lObstacles;

	cout << "Parsing obstacles...!!!"<<endl;
	Tokenizer lLineTokenizer(inLine, ";");
	while (lLineTokenizer.NextToken())
	{
		std::string lXYStr = lLineTokenizer.GetToken();
		Tokenizer lXYVTokenizer(lXYStr, ",");
		lXYVTokenizer.NextToken();
		double lX = atof(lXYVTokenizer.GetToken().c_str());
		lXYVTokenizer.NextToken();
		double lY = atof(lXYVTokenizer.GetToken().c_str());

		vector<double> lObstacle;
		lObstacle.push_back(lX);
		lObstacle.push_back(lY);
		lObstacles.push_back(lObstacle);
	}
	cout<<"Parsed size: "<<lObstacles.size()<<endl;
	return lObstacles;
}

vector<vector<double> > Parser::parseObstacles(const char* FILE_NAME)
{
	vector<vector<double> > lObstacles;
    ifstream lIFSIndividuals(FILE_NAME, ifstream::in);
    while (lIFSIndividuals.good())
        {
            string lReadLine;
            getline(lIFSIndividuals, lReadLine);
            vector<double> lObstacle;
            if(lReadLine=="")
            {
                break;
            }
            else
			{
                Tokenizer lTokenizer(lReadLine, ",");
                while (lTokenizer.NextToken())
                {
                    string lToken = lTokenizer.GetToken();
                    double lParameter = atof(lToken.c_str());
                    lObstacle.push_back(lParameter);
                }
            }

            lObstacles.push_back(lObstacle);

        }
    return lObstacles;
}
vector<vector<double> > Parser::loadObstacles(std::string inFilename)
{
	cout << "Loading obstacles...!!!" << endl;
	vector<vector<double> > lObstacles;
	ifstream lIFStream(inFilename.c_str());
	std::string lObstaclesString;
	if (lIFStream.good()) {
		std::getline(lIFStream, lObstaclesString);
	} else {
		cout << "Cannot read obstacles from the file!" << endl;
		return lObstacles;
	}

	return parseObstaclesCont(lObstaclesString);
}

vector<vector<Action> > Parser::parseControlActions(const char* inLine)
{
	vector<vector<Action> > lEntireControlActionSet;

	cout << "Parsing Control Action...!!!"<<endl;
	ifstream lIFSIndividuals(inLine, ifstream::in);
	while (lIFSIndividuals.good())
	{
		vector<Action> lActionSet;
		lActionSet.clear();
		std::string lNewActionsLine;
		getline(lIFSIndividuals, lNewActionsLine);
		if(lNewActionsLine=="")
        {
            break;
        }
		else
		{
			Tokenizer lActionProfileTokenizer(lNewActionsLine, ";");
			while (lActionProfileTokenizer.NextToken())
			{
				std::string lXYStr = lActionProfileTokenizer.GetToken();
				Tokenizer lXYVTokenizer(lXYStr, ",");
				lXYVTokenizer.NextToken();
				double lX = atof(lXYVTokenizer.GetToken().c_str());
				lXYVTokenizer.NextToken();
				double lY = atof(lXYVTokenizer.GetToken().c_str());
				lXYVTokenizer.NextToken();
				double lPSI = atof(lXYVTokenizer.GetToken().c_str());
				lXYVTokenizer.NextToken();
				double lCost = atof(lXYVTokenizer.GetToken().c_str());
				vector<double> lAction;
				lAction.push_back(lX);
				lAction.push_back(lY);
				lAction.push_back(lPSI);
				Action lNewAction(lAction,lCost);
			
				lActionSet.push_back(lNewAction);
			}
			lEntireControlActionSet.push_back(lActionSet);
		}
	}
	
	return lEntireControlActionSet;
}

vector<vector<Action> > Parser::parseIntermediateControlActions(const char* inLine)
{
	vector<vector<Action> > lEntireControlActionSet;

	cout << "Parsing Intermediate Control Action...!!!"<<endl;
	ifstream lIFSIndividuals(inLine, ifstream::in);
	while (lIFSIndividuals.good())
	{
		vector<Action> lActionSet;
		lActionSet.clear();
		std::string lNewActionsLine;
		getline(lIFSIndividuals, lNewActionsLine);
		if(lNewActionsLine=="")
        {
            break;
        }
		else
		{
			Tokenizer lActionProfileTokenizer(lNewActionsLine, ";");
			while (lActionProfileTokenizer.NextToken())
			{
				std::string lXYStr = lActionProfileTokenizer.GetToken();
				Tokenizer lXYVTokenizer(lXYStr, ",");
				lXYVTokenizer.NextToken();
				double lX = atof(lXYVTokenizer.GetToken().c_str());
				lXYVTokenizer.NextToken();
				double lY = atof(lXYVTokenizer.GetToken().c_str());
				lXYVTokenizer.NextToken();
				double lPSI = atof(lXYVTokenizer.GetToken().c_str());
				double lCost = 0;
				vector<double> lAction;
				lAction.push_back(lX);
				lAction.push_back(lY);
				lAction.push_back(lPSI);
				Action lNewAction(lAction,lCost);
				//cout<<lX<<","<<lY<<','<<lPSI<<endl;
				lActionSet.push_back(lNewAction);
			}
			lEntireControlActionSet.push_back(lActionSet);
		}
	}
	
	return lEntireControlActionSet;
}

vector<vector<double> > Parser::loadObstaclesCenter(std::string inFilename)
{
	cout << "Loading obstacles...!!!" << endl;
	vector<vector<double> > lObstacles;
	ifstream lIFStream(inFilename.c_str());
	std::string lObstaclesString;
	if (lIFStream.good()) {
		std::getline(lIFStream, lObstaclesString);
	} else {
		cout << "Cannot read obstacles from the file!" << endl;
		return lObstacles;
	}

	return parseObstaclesCenter(lObstaclesString);
}

vector<vector<double> > Parser::parseObstaclesCenter(std::string& inLine)
{
	vector<vector<double> > lObstacles;

	cout << "Parsing obstacles...!!!"<<endl;
	Tokenizer lLineTokenizer(inLine, ";");
	while (lLineTokenizer.NextToken())
	{
		std::string lXYStr = lLineTokenizer.GetToken();
		Tokenizer lXYVTokenizer(lXYStr, ",");
		lXYVTokenizer.NextToken();
		double lX = atof(lXYVTokenizer.GetToken().c_str());
		lXYVTokenizer.NextToken();
		double lY = atof(lXYVTokenizer.GetToken().c_str());
		lXYVTokenizer.NextToken();
		double lRadius = atof(lXYVTokenizer.GetToken().c_str());

		vector<double> lObstacle;
		lObstacle.push_back(lX);
		lObstacle.push_back(lY);
		lObstacle.push_back(lRadius);
		lObstacles.push_back(lObstacle);
	}
	cout<<"Parsed Obstacle Centers size: "<<lObstacles.size()<<endl;
	return lObstacles;
}
