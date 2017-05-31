/*
 * IRSensor.h
 *
 *  Created on: May 8, 2017
 *      Author: marvin
 */

#ifndef LIBRARIES_IRSENSOR_IRSENSOR_H_
#define LIBRARIES_IRSENSOR_IRSENSOR_H_
#include <Arduino.h>
#include <LightChrono.h>
//#include "MedianFilter.h"

	typedef enum {
	  SHARP_DX, //2D120X
	  SHARP_Ya, //2Y0A21
	  SHARP_YA //2Y0A02
	} SHARP;
typedef enum corrType{
	LINEAR,
	EXPONENTIAL
}cType;
class IRSensor {

public:
	IRSensor(SHARP type, int avgNum);
	virtual ~IRSensor();
	float getValue(bool useFilter = false);
	void setup(float a1, float a2);

private:
	SHARP type_;
	LightChrono chrono_;
	float oldDist_;
	int pin_;
	float corrParam1_;
	float corrParam2_;
	float getCorrectLinear(float val);
	float getCorrectExp(float val);
	float readSensor(SHARP which_one, int which_analog_pin);
	float movingAverFilter(float curDistance);
	float movingMedianFilter(float curDistance);
	void bubbleSort();
	int numValues;
	float recordDistances[10];
	//MedianFilter medianFilter(10,0);
};

#endif /* LIBRARIES_IRSENSOR_IRSENSOR_H_ */

