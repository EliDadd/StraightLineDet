#include "notLine.h"


using namespace std;

void notLine::addPoint(float xVal, float yVal){
	x.pushback(xVal);
	y.pushback(yVal);
}
void setEndpoints(){
	end1.setCart(x[0], y[0]);
	end2.setCart(x[x.size()-1], y[y.size()-1]);
}

int notLine::getSize(){
	return x.size();
}
	
