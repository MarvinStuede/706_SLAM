/*
 * UltraSonicSensor.h
 *
 *  Created on: May 9, 2017
 *      Author: marvin
 */

#include <NewPing.h>
#include <LightChrono.h>
#ifndef ULTRASONICSENSOR_H_
#define ULTRASONICSENSOR_H_

#define TRIGGER_PIN  48  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     49  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 400

class UltraSonicSensor {
public:
	UltraSonicSensor();
	virtual ~UltraSonicSensor();
	void setup();
	float getDistance();
private:
	LightChrono chrono_;
	NewPing sonar_;
	float oldDist_;
};

#endif /* ULTRASONICSENSOR_H_ */
