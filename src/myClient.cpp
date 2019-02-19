#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "math.h"
#include <vector>
#include "incrementalLine.h"

#define RAD2DEG(x) ((x)*180./M_PI)
#define POLAR2XCART(x, y) ((x)*cos((y)*M_PI/180.)) //get the x component when given a distance and angle in degrees
#define POLAR2YCART(x, y) ((x)*sin((y)*M_PI/180.)) //get the y component when given a distance and angle in degrees

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	int count = scan->scan_time / scan->time_increment;
	ROS_INFO("Testing %s[%d]:", scan->header.frame_id.c_str(), count);
	ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
	std::vector <float> xVal;
	std::vector <float> yVal;
	float XRange[scan->ranges.size()];
	float YRange[scan->ranges.size()];
	for(int i = 0; i < count; i++) {
		float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
	//	ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
		if ((isinf(POLAR2XCART(scan->ranges[i], degree))) == 0){
			xVal.push_back(POLAR2XCART(scan->ranges[i], degree));
			yVal.push_back(POLAR2YCART(scan->ranges[i], degree));
		}
		

	}
	ROS_INFO("Starting findLine...");
	findLine(xVal, yVal);
//	for(int i = 0; i < xVal.size(); i++){
//		ROS_INFO(" Testing: X = %f, Y = %f", xVal[i], yVal[i]);
//	}
}


void findLine(vector <float> xReal, vector <float> yReal){
	vector <line> myLines;
	vector <line> fakeLines;
	line tempFake;
	line tempLine;
//	int numLines = 0;			// how many lines detected
	int scopeSize = 0;			// keeps track of where the line breaks
	int i = 0;					// iterator
	float distToLine = 0;		// distance from a point to a line
	bool twoPointLine = false;  // checks to see if the line has only 2 points
	float distToPoint = 0;			// distance from point to point (If to big, create new line
	bool needNewFLine = true;

	//adjustable variables
	//Best Vals
	//pointThreshold:	 .02
	//pointDistThresh:	 .4
	float pointThreshold = 0.02;		//distance between the point and line threshold
	float pointDistThresh = .4;		//distance between point to point in a line (if it exceeds this threshold it will make a new line)

	while (i < xReal.size()) {
		for (i; i < xReal.size(); i++) {
			if (i == scopeSize) {	//adding first point to a line
				
				tempLine.addPoint(xReal[i], yReal[i]);
			}
			else if (i == scopeSize +1){							//checking the second point
				distToPoint = pt2PtDist(xReal[i], yReal[i], xReal[i-1], yReal[i-1]);	//checks the distance between the first and second point
				if(distToPoint < pointDistThresh){					//if they are close enough, add second point to line
					tempLine.addPoint(xReal[i], yReal[i]);
				}
				else{									//otherwise send to 'not line' array
					cout << "distToLine: " << distToLine << "    distToPoint: " << distToPoint << endl;
                                        cout << "checking 2nd point" << endl;

					if (needNewFLine == true){					//checks to see if there needs to be a new group of bad points
						tempFake = tempLine;					//if yes, it will make a new array for points
						fakeLines.push_back(tempFake);
						tempFake.clearLine();
						needNewFLine = false;
					}
					else{ 
						fakeLines[fakeLines.size()-1].mergeLines(tempLine);	//otherwise it will add to the previous array of points

					}

						
					scopeSize = i;
					distToPoint = 0;
					twoPointLine = true;
					break;
				}
			}


			
			else if (i == scopeSize + 2) {

				
				tempLine.setFloats();
				distToPoint = pt2PtDist(xReal[i], yReal[i], xReal[i-1], yReal[i-1]);
				distToLine = tempLine.findDist(xReal[i], yReal[i]);


				//if the point is within 1 cm to the line, add the point, otherwise end the line
				if (distToLine < pointThreshold) {
					if(distToPoint <= pointDistThresh){
						tempLine.addPoint(xReal[i], yReal[i]);
						myLines.push_back(tempLine);
					}
					else {
						cout << "distToLine: " << distToLine << "    distToPoint: " << distToPoint << endl;
                                                cout << "checking 3rd point" << endl;

						if (needNewFLine == true){
                                               		tempFake = tempLine;
                                                	fakeLines.push_back(tempFake);
                                                	tempFake.clearLine();
							needNewFLine = false;
                                        	}
                                        	else{
							fakeLines[fakeLines.size() - 1].mergeLines(tempLine);
						}
						scopeSize = i;
						break;
					}
				}
				else {
					cout << "distToLine: " << distToLine << "    distToPoint: " << distToPoint << endl;
                                        cout << "checking 3rd point" << endl;

					if (needNewFLine == true){
                                        	tempFake = tempLine;
                                        	fakeLines.push_back(tempFake);
                                        	tempFake.clearLine();
						needNewFLine = false;
                                        }
                                        else{
                                        	fakeLines[fakeLines.size() - 1].mergeLines(tempLine);
                                        }


					scopeSize = i;
					twoPointLine = true;
					break;

				}
			}
			else if (i < scopeSize + 11) {
                                myLines[myLines.size()-1].setFloats();
                                distToPoint = pt2PtDist(xReal[i], yReal[i], xReal[i-1], yReal[i-1]);
                                distToLine = myLines[myLines.size()-1].findDist(xReal[i], yReal[i]);

                                if (distToLine < pointThreshold) {
                                        if(distToPoint <= pointDistThresh){

                                                myLines[myLines.size()-1].addPoint(xReal[i], yReal[i]);
                                        }
                                        else{
						cout << "distToLine: " << distToLine << "    distToPoint: " << distToPoint << endl;
						cout << "checking 10 or less points" << endl;
						if (needNewFLine == true){
                                                        tempFake = myLines[myLines.size()-1];
                                                        fakeLines.push_back(tempFake);
                                                        tempFake.clearLine();
							needNewFLine = false;
                                                }
                                                else{
                                                        fakeLines[fakeLines.size() - 1].mergeLines(myLines[myLines.size()-1]);
                                                }

						myLines.erase(myLines.begin() + myLines.size() -1);


                                                scopeSize = i;
                                                break;
                                        }
                                }
                                else {
					cout << "distToLine: " << distToLine << "    distToPoint: " << distToPoint << endl;
                                        cout << "checking 10 or less points" << endl;

					if (needNewFLine == true){
                                        	tempFake = myLines[myLines.size()-1];
                                        	fakeLines.push_back(tempFake);
                                        	tempFake.clearLine();
                                        	needNewFLine = false;
                                        }
                                        else{
                                        	fakeLines[fakeLines.size() - 1].mergeLines(myLines[myLines.size()-1]);
                                        }

					myLines.erase(myLines.begin() + myLines.size() - 1);

                                        scopeSize = i;
                                        break;
                                }
                //              cout << "other points are good\n";
                        }


			else {
			//	cout << "testing other points\n";
				needNewFLine = true;
				myLines[myLines.size()-1].setFloats();
			//	cout << "checking pt2PtDist\n";
				distToPoint = pt2PtDist(xReal[i], yReal[i], xReal[i-1], yReal[i-1]);
				distToLine = myLines[myLines.size()-1].findDist(xReal[i], yReal[i]);

				//if the point is within 1 cm to the line, add the point, otherwise end the line
				if (distToLine < pointThreshold) {
					if(distToPoint <= pointDistThresh){

						myLines[myLines.size()-1].addPoint(xReal[i], yReal[i]);
					}
					else{
						
						scopeSize = i;
						break;
					}
				}
				else {
					scopeSize = i;
					break;
				}
		//		cout << "other points are good\n";
			}
		}

		tempLine.clearLine();
		if (twoPointLine == false) {
		//delete this line
		}
		twoPointLine = false;

	}

	for (int y = 0; y < fakeLines.size(); y++){
		float fakeLineLength = pt2PtDist(fakeLines[y].getEndPtX1(),fakeLines[y].getEndPtY1(), fakeLines[y].getEndPtX2(), fakeLines[y].getEndPtY2());
		cout << "fake line " << y+1 << endl;
		fakeLines[y].printLine();
		cout << endl << fakeLineLength << endl << endl;
		if (( fakeLineLength < 0.04 ) && ( fakeLineLength > 0.01 )){
			cout << "Candle detected as fake Line number: " << y << endl;
		}
				

	}
        

	cout << endl << endl;

//	cout<<"Lines made\n";
	if (myLines.size() == 0) {cout << "about to access a negative line size" << endl;}
	myLines[myLines.size()-1].mergeLines(myLines[0]);
	myLines[myLines.size()-1].setFloats();
	myLines.erase(myLines.begin());

	float distToEnd1 = 0;
	float distToEnd2 = 0;
	for (int u = 0; u < fakeLines.size(); u++){
		for (int o = 0; o < fakeLines[u].lineSize(); o++){
			for ( int p = 0; p < myLines.size(); p++){
				distToLine = myLines[p].findDist(fakeLines[u].getXPoint(o),fakeLines[u].getYPoint(o));
				distToEnd1 = pt2PtDist(fakeLines[u].getXPoint(o), fakeLines[u].getYPoint(o), myLines[p].getEndPtX1(), myLines[p].getEndPtY1());
			}
		}

	}
	/*for (int j = 0; j < myLines.size(); j++){

		tempLine.addPoint(myLines[j].getEndPtX1(), myLines[j].getEndPtY1());
		tempLine.addPoint(myLines[j].getEndPtX2(), myLines[j].getEndPtY2());
		tempLine.setFloats();
		myLines[j].setSlope(tempLine.getSlope());
		myLines[j].setIntercept(tempLine.getIntercept());
		tempLine.clearLine();
	}*/

	for (int j = 0; j < myLines.size(); j++) {
                float ang1, ang2, rad1, rad2;
		ang1 = myLines[j].endPAngle(1);
		ang2 = myLines[j].endPAngle(2);
		rad1 = myLines[j].endPRad(1);
		rad2 = myLines[j].endPRad(2);
                cout << endl << "Line: " << j + 1 << " size: " << myLines[j].lineSize();
                cout <<" Slope: " << myLines[j].getSlope() << " Intercept: " << myLines[j].getIntercept();
                cout << " Endpoints: (" << myLines[j].getEndPtX1() << ", " << myLines[j].getEndPtY1();
                cout << ") Angle: " << ang1;
                cout << " Distance: " << rad1 << endl;
                cout << "          (" << myLines[j].getEndPtX2() << ", " << myLines[j].getEndPtY2();
                cout << ") Angle:" <<  ang2;
                cout << " Distance: " << rad2 << endl;

		//              myLines[j].printLine();
        }

	cout << endl << endl;

	for (int g = 0; g < myLines.size(); g++){
		for (int h = 0; h < g; h++){
			if ( canMerge(myLines[g], myLines[h]) == true){
				myLines[h].mergeLines(myLines[g]);
				myLines[h].setFloats();
				myLines.erase(myLines.begin() + g);
				g=0;
				h=0;
			}
		}
	}

	for (int j = 0; j < myLines.size(); j++) {
                float ang1, ang2, rad1, rad2;
                ang1 = myLines[j].endPAngle(1);
                ang2 = myLines[j].endPAngle(2);
                rad1 = myLines[j].endPRad(1);
                rad2 = myLines[j].endPRad(2);
                cout << endl << "Line: " << j + 1 << " size: " << myLines[j].lineSize();
                cout <<" Slope: " << myLines[j].getSlope() << " Intercept: " << myLines[j].getIntercept();
                cout << " Endpoints: (" << myLines[j].getEndPtX1() << ", " << myLines[j].getEndPtY1();
                cout << ") Angle: " << ang1;
                cout << " Distance: " << rad1 << endl;
                cout << "          (" << myLines[j].getEndPtX2() << ", " << myLines[j].getEndPtY2();
                cout << ") Angle:" <<  ang2;
                cout << " Distance: " << rad2 << endl;

                //              myLines[j].printLine();
        }

};
float pt2PtDist(float x1, float y1, float x2, float y2){
	float dist;
	dist = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
	return dist;
}

bool canMerge(line a, line b){
	float slopeThresh = 7;
        float interceptThresh = 7;
        float sizeThresh = 100; 
	float distThresh = .03;
	float myDist;
	float tempDist;
	line temp1;
	line temp2;

	temp1.addPoint(a.getEndPtX1(), a.getEndPtY1());
	temp1.addPoint(a.getEndPtX2(), a.getEndPtY2());
	temp2.addPoint(b.getEndPtX1(), b.getEndPtY1());
	temp2.addPoint(b.getEndPtX2(), b.getEndPtY2());
	temp1.setFloats();
	temp2.setFloats();

	
	myDist = pt2PtDist(a.getEndPtX1(), a.getEndPtY1(), b.getEndPtX1(), b.getEndPtY1());

        tempDist = pt2PtDist(a.getEndPtX2(), a.getEndPtY2(), b.getEndPtX2(), b.getEndPtY2());
        if (tempDist < myDist) {myDist = tempDist;}

        tempDist = pt2PtDist(a.getEndPtX1(), a.getEndPtY1(), b.getEndPtX2(), b.getEndPtY2());
        if (tempDist < myDist) {myDist = tempDist;}

        tempDist = pt2PtDist(a.getEndPtX2(), a.getEndPtY2(), b.getEndPtX1(), b.getEndPtY1());
        if (tempDist < myDist) {myDist = tempDist;}
	
	if(myDist < distThresh){
		if((temp1.getSlope()*temp2.getSlope() > -1.7) && (temp1.getSlope()*temp2.getSlope() < -.3))
			return false;
	
		else
			return true;
	}
	else
		return false;



/*	if ((abs(temp1.getSlope()-temp2.getSlope()) < slopeThresh)&&(abs(temp1.getIntercept()-temp2.getIntercept()) < interceptThresh)){
		myDist = pt2PtDist(a.getEndPtX1(), a.getEndPtY1(), b.getEndPtX1(), b.getEndPtY1());

		tempDist = pt2PtDist(a.getEndPtX2(), a.getEndPtY2(), b.getEndPtX2(), b.getEndPtY2());
		if (tempDist < myDist) {myDist = tempDist;}

		tempDist = pt2PtDist(a.getEndPtX1(), a.getEndPtY1(), b.getEndPtX2(), b.getEndPtY2());
		if (tempDist < myDist) {myDist = tempDist;}

		tempDist = pt2PtDist(a.getEndPtX2(), a.getEndPtY2(), b.getEndPtX1(), b.getEndPtY1());
		if (tempDist < myDist) {myDist = tempDist;}

		if(myDist < distThresh){return true;}

		
		else{return false;}





	}
	else{return false;}
*/
}

void line::setSlope(float s){
	slope = s;
};

void line::setIntercept(float i){
	intercept = i;
};

float line::getXPoint(int point){
	return x[point];
};

float line::getYPoint(int point){
	return y[point];
};

void line::setFloats() {
	float xAvg = 0, yAvg = 0;
	for (int i = 0; i < x.size(); i++) {
		xAvg += x[i];
		yAvg += y[i];
	};
	xAvg /= (x.size());
	yAvg /= (y.size());

	float num = 0, denum = 0;

	for (int i = 0; i < x.size(); i++) {
		num += (x[i] - xAvg)*(y[i] - yAvg);
		denum += pow(x[i] - xAvg, 2);
	};
	slope = num / denum;
	intercept = yAvg - slope * xAvg;
	setEndpts(x[0], y[0], x[x.size()-1], y[y.size()-1]);
};

float line::getIntercept() {

	return intercept;
};

float line::getSlope() {

	return slope;
};

void line::addPoint(float xVal, float yVal) {
	x.push_back(xVal);
	y.push_back(yVal);
};

float line::findDist(float xPoint, float yPoint) {
	float a, b, c;
	a = slope;
	b = -1;
	c = intercept;
	float num, denum;
	num = abs(a*xPoint + b * yPoint + c);
	denum = sqrt(pow(a, 2) + pow(b, 2));
	float d;
	d = num / denum;

	return d;
};

void line::clearLine() {
	x.clear();
	y.clear();
	slope = 0;
	intercept = 0;
};

void line::printLine() {
	for (int i = 0; i < x.size(); i++) {
		float angle;
		float R;
		float t = atan2(y[i], x[i]) * 180 / 3.14159;
        	angle = t>0 ? t : t + 360; // polar vals range 0:360
		R = pow(x[i]*x[i] + y[i]*y[i], 0.5);
		std::cout << angle << "   " << R << endl;
	}
};

float line::endPAngle(int num){
	if(num == 1) {return end1.findAngle();}
	else if (num == 2) {return end2.findAngle();}
	else {return 0;}
}

float line::endPRad(int num){
	if(num == 1) {return end1.findRad();}
	else if (num == 2) {return end2.findRad();}
	else {return 0;}
}



float line::lineSize() {
	return x.size();
}

void line::setEndpts(float x1, float y1, float x2, float y2){
	end1.setCart(x1, y1);
	end2.setCart(x2, y2);
}
float line::getEndPtX1(){
	return end1.getX();
}
float line::getEndPtY1(){
	return end1.getY();
}
float line::getEndPtX2(){
	return end2.getX();
}
float line::getEndPtY2(){
	return end2.getY();
}
void line::mergeLines(line a) {//line a gets merged into the main line
	for(int i = 0; i < a.lineSize(); i++){
		x.push_back(a.getXPoint(i));
	     	y.push_back(a.getYPoint(i));
		//x.insert(x.begin(), a.getXPoint(a.lineSize() - i));
		//y.insert(y.begin(), a.getYPoint(a.lineSize() - i));
		//still have to delete the line a in findLine
	}
	a.clearLine();

}

void endpoint::setCart(float xIn, float yIn){
	x = xIn;
	y = yIn;
}
float endpoint::getX(){
	return x;
}
float endpoint::getY(){
	return y;
}
float endpoint::findAngle(){
	float angle;
        float t = atan2(y, x) * 180 / 3.14159;
        angle = t>0 ? t : t + 360; // polar vals range 0:360
	return angle;
}
float endpoint::findRad(){
	float rad;
	rad = pow(x*x + y*y, 0.5);
	return rad;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "rplidar_node_client");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

	ros::spin();

	return 0;
}

