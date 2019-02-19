#ifndef NOT_LINE_INCLUDE
#define NOT_LINE_INCLUDE

#include <iostream>
#include <cmath>
#include <vector>
#include <utility>
#include "endpoint.h"

using namespace std;

class notLine {
public:
	void addPoint(float xVal, float yVal);
	void setEndpoints();
	int getSize();
private:
	endpoint end1;
	endpoint end2;
	vector <float> x;
	vector <float> y;

}


#endif
