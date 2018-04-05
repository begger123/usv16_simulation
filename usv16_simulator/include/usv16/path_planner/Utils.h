#pragma once

//#include "math3d.h"

#define	PI 3.141592653589793238

class Utils
{
	public:
		static double degToRad(double inDegrees);
		static double radToDeg(double inRadians);
		static double norm(double inX1, double inY1, double inX2, double inY2);
		static double round(double inValue);
		static double angleBetweenVec(double inX1, double inY1, double inX2, double inY2);
};