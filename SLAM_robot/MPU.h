/*
 * MPU.h
 *
 *  Created on: May 8, 2017
 *      Author: marvin
 */

#ifndef LIBRARIES_MPU_MPU_H_
#define LIBRARIES_MPU_MPU_H_
#include <Wire.h>
#include <Arduino.h>
#include <LightChrono.h>
#include "MPU9250.h"
#include "quaternionFilters.h"
#define AHRS true         // Set to false for basic data read
#define SerialDebug true  // Set to true to get Serial output for debugging

class MPU {
public:
	MPU();
	virtual ~MPU();
	void setup();
	void getMag(float *data);
	void getGyro(float *data);
	void getRPY(float &roll, float &pitch, float &yaw);
	void readRegisters();

private:
	MPU9250 mpu;
	float magBias_[3];
	void readMagData(int16_t *magData);
	void calibrate();
	void updateQuaternions();

};



#endif /* LIBRARIES_MPU_MPU_H_ */

