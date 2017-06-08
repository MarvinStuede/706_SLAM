/*
 * Mapping.cpp
 *
 *  Created on: May 25, 2017
 *      Author: seungmin
 */

#include "Mapping.h"


Mapping::Mapping() {
};

Mapping::~Mapping() {
	// TODO Auto-generated destructor stub
}

void Mapping::initialise(float sonarDist, float irDist)
{
	xPos[0] = irDist - centriodToSideIR_dist;
	yPos[0] = centroidToBack_dist;
	numPoints++;
	x_measure = irDist;
	y_measure = sonarDist;
	xPosNow_ = irDist - centriodToSideIR_dist;
	yPosNow_ = centroidToBack_dist;
	xPosOld_ = xPosNow_;
	yPosOld_ = yPosNow_;
}

void Mapping::savePoint(float sonarDist, float irDist, int direction)
{
	//Moving parallel to y axis positive direction
	//if(numPoints < arrayLength_){
		if (direction == 1) {
			//xPos[numPoints] = irDist - centriodToSideIR_dist;
			//yPos[numPoints] = (y_measure - sonarDist) + yPos[numPoints -1];
			xPosNow_ = irDist - centriodToSideIR_dist;
			yPosNow_ += (y_measure - sonarDist);
			x_measure = irDist;
			y_measure = sonarDist;
		}
		//Moving parallel to x axis positive direction
		else if (direction == 2) {
			//		xPos[numPoints] = (x_measure - sonarDist) + xPos[numPoints - 1];
			//		yPos[numPoints] = irDist - centriodToSideIR_dist + y_ref;
			xPosNow_ += (x_measure - sonarDist);
			yPosNow_ -= irDist - centriodToSideIR_dist - y_measure;
			x_measure = sonarDist;
			y_measure = irDist;
		}
		//Moving parallel to y axis negative direction
		else if (direction == 3) {
			//		xPos[numPoints] = irDist - centriodToSideIR_dist + x_ref;
			//		yPos[numPoints] = yPos[numPoints - 1] - (y_measure - sonarDist);
			xPosNow_ -= irDist - centriodToSideIR_dist -x_measure;
			yPosNow_ -= (y_measure - sonarDist);
			x_measure = irDist;
			y_measure = sonarDist;
		}
		//Moving parallel to x axis negative direction
		else if (direction == 4) {
//			xPos[numPoints] = xPos[numPoints - 1] - (x_measure - sonarDist);
//			yPos[numPoints] = irDist - centriodToSideIR_dist;
			xPosNow_ -= (x_measure - sonarDist);
			yPosNow_ = irDist - centriodToSideIR_dist;
			x_measure = sonarDist;
			y_measure = irDist;
		}
		yPosOld_ = yPosNow_;
		xPosOld_ = xPosNow_;
		//numPoints++;
	//}
}

void Mapping::robotTurned(float sonarDist, float irDist, int direction)
{
	if (direction == 1) {
		x_measure = irDist;
		y_measure = sonarDist;
	}
	else if (direction == 2) {
		x_measure = sonarDist;
		y_measure = irDist;
		y_ref = yPosOld_;
	}
	else if (direction == 3) {
		x_measure = irDist;
		y_measure = sonarDist;
		x_ref = xPosOld_;
	}
	else if (direction == 4){
		x_measure = sonarDist;
		y_measure = irDist;
	}
}

void Mapping::sendData() {
	for (int i = 0; i<=numPoints;i++){
		Serial.print(xPos[i]);
		Serial.print(" ");
		Serial.print(yPos[i]);
		Serial.print(" ");
		Serial.println();
		delay(2);
	}
}

void Mapping::sendCurrentPoint(int turnCnt) {
//	Serial.print(xPosNow_);
//	Serial.print(" ");
//	Serial.print(yPosNow_);
//	Serial.print(" ");
//	Serial.print(turnCnt * -90.);
//	Serial.print(" ");
//	Serial.println();

	Serial1.print(xPosNow_);
	Serial1.print(" ");
	Serial1.print(yPosNow_);
	Serial1.print(" ");
	Serial1.print(turnCnt * -90.);
	Serial1.print(" ");
	Serial1.println();
}
/*
void firstTurn(float secondSonarIn) {
 // yRefSonar = secondSonarIn;
}

void saveInfo(float sonarInput, bool isX) {
  if(isX == 1) {
//    xPos = xRefSonar - sonarInput;
  }
  else  {
 //   yPos = yRefSonar - sonarInput;
  }
}

void sendInfo() {

}

 */
