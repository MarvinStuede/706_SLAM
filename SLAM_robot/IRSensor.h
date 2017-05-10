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
	typedef enum {
	  SHARP_DX, //2D120X
	  SHARP_Ya, //2Y0A21
	  SHARP_YA //2Y0A02
	} SHARP;
class IRSensor {

public:
	IRSensor(SHARP type, int avgNum);
	virtual ~IRSensor();
	int getVal();
	void setup();
private:
	SHARP type_;
	LightChrono chrono_;
	float oldDist_;
	int pin_;
	int read(SHARP which_one, int which_analog_pin);
  float movingAverFilter(float curDistance);
  int numValues;
  float recordDistances[10];
};

#endif /* LIBRARIES_IRSENSOR_IRSENSOR_H_ */
