#include "path_planner/Utils.h"
#include "math.h"

double Utils::degToRad(double inDeg) 
{
	double lResult = inDeg * (PI / 180.0);
	return lResult;
}

double Utils::radToDeg(double inRad)
{
	double lResult = (double) (inRad * (180.0 / PI));
	return lResult;
}
double Utils::round(double inValue)
{
	double fractpart, intpart;
	fractpart = modf (inValue , &intpart);
	if (fractpart>=.5)
		return inValue>=0?ceil(inValue):floor(inValue);
	else
		return inValue<0?ceil(inValue):floor(inValue);
}

double Utils::norm(double inX1, double inY1, double inX2, double inY2)
{
	return sqrt((inX1-inX2)*(inX1-inX2) + (inY1-inY2)*(inY1-inY2));
}

double Utils::angleBetweenVec(double inX1, double inY1, double inX2, double inY2)
{
	double vec1Mag = norm(inX1, inY1,0,0);
	double vec2Mag = norm(inX2, inY2,0,0);
	double vecMultiplication = inX1*inX2 + inY1*inY2;
	double c = (vecMultiplication) / (vec1Mag * vec2Mag);
	
	if (c > 1) {
		c = 1;
	} else if (c < -1) {
		c = -1;
	}
 
	if (((inX1 * inY2) - (inY1 * inX2)) < 0) {
		return -acos(c);
	} else {
		return acos(c);
	}
}
