#ifndef ENDPOINT_INCLUDE
#define ENDPOINT_INCLUDE

#include <iostream>
#include <cmath>
#include <vector>
#include <utility>

using namespace std;

class endpoint {
public:
	void setCart(float xIn, float yIn);
	float getX();
	float getY();
	float findAngle();
	float findRad();

private:
	float x;
	float y;
};


#endif


