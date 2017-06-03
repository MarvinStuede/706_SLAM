/*
 * Mapping.h
 *
 *  Created on: May 25, 2017
 *      Author: seungmin
 */

#include <Arduino.h>
#include <LightChrono.h>

#ifndef MAPPING_H_
#define MAPPING_H_

class Mapping {

public:
	Mapping();
	virtual ~Mapping();
	void initialise(float sonarDist, float irDist);
	void savePoint(float sonarDist, float irDist, int direction);
	void robotTurned(float sonarDist, float irDist, int direction);
	//void saveInfo(float sonarInput, bool isX);
	//void firstTurn();
	//void sendInfo();
private:
	float xPos[500];
	float yPos[500];
	float x_measure = 0;
	float y_measure = 0;
	float x_ref = 0;
	float y_ref = 0;
	int numPoints = 0;
	float yMaxDist = 0;
	float xMaxDist = 0;
	float xRefSonar;
	float yRefSonar;
	static constexpr float centriodToSonar_dist = 0.0863;  //**********************NEED TO INPUT CORRECT CONSTANT VALUES*********************
	static constexpr float centriodToSideIR_dist = 0.0763;
	static constexpr float centroidToBack_dist = 0.01;
};

#endif /* MAPPING_H_ */
