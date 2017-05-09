/*
 * UltraSonicSensor.cpp
 *
 *  Created on: May 9, 2017
 *      Author: marvin
 */

#include "UltraSonicSensor.h"

UltraSonicSensor::UltraSonicSensor() : sonar_(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE){
	oldDist_ = 0;
}

UltraSonicSensor::~UltraSonicSensor() {

}

void UltraSonicSensor::setup() {
	chrono_.start();
}

float UltraSonicSensor::getDistance() {

	if(chrono_.elapsed() > 50)//Wait 50ms between measurements
	{
		oldDist_ = sonar_.ping_cm();
		chrono_.restart();
	}
	return oldDist_;
}
