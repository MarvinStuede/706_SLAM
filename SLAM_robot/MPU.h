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
#include "MPU9150.h"

class MPU {
public:
	MPU();
	virtual ~MPU();
	void setup();
	void getMag(float *data);
	void getGyro(float *data);

private:
	MPU9150 mpu;
	float magBias_[3];
	void readMagData(int16_t *magData);
	void calibrate();

};



#endif /* LIBRARIES_MPU_MPU_H_ */
