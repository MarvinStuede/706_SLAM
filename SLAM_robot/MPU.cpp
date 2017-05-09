/*
 * MPU.cpp
 *
 *  Created on: May 8, 2017
 *      Author: marvin
 */

#include "MPU.h"



MPU::MPU() {
	// TODO Auto-generated constructor stub

}

MPU::~MPU() {
	// TODO Auto-generated destructor stub
}

void MPU::setup() {
	mpu.initialize();
	mpu.setFullScaleGyroRange(0x00); //Set max range to +- 250 deg/s
	calibrate();
}

void MPU::calibrate()
{
	uint16_t ii = 0, sample_count = 0;
	int32_t mag_bias[3] = {0, 0, 0};
	int16_t mag_max[3] = {0, 0, 0}, mag_min[3] = {0, 0, 0}, mag_temp[3] = {0, 0, 0};
	Serial.println("Mag Calibration: Wave device in a figure eight until done!");
	delay(4000);
	sample_count = 64;
	for(ii = 0; ii < sample_count; ii++) {
	readMagData(mag_temp); // Read the mag data
	for (int jj = 0; jj < 3; jj++) {
	if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
	if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
	}
	delay(135); // at 8 Hz ODR, new mag data is available every 125 ms
	}
	 Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
	 Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
	 Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);
	mag_bias[0] = (mag_max[0] + mag_min[0])/2; // get average x mag bias in counts
	mag_bias[1] = (mag_max[1] + mag_min[1])/2; // get average y mag bias in counts
	mag_bias[2] = (mag_max[2] + mag_min[2])/2; // get average z mag bias in counts
	magBias_[0] = (float) mag_bias[0]; // save mag biases in G for main program
	magBias_[1] = (float) mag_bias[1];
	magBias_[2] = (float) mag_bias[2];
	Serial.println("Mag Calibration done!");

}

void MPU::readMagData(int16_t* magData) {
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
	int16_t mx, my, mz;
	mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
	magData[0] = mx;
	magData[1] = my;
	magData[2] = mz;
}

void MPU::getMag(float* data) {
	int16_t readData[3];
	readMagData(readData);
	data[0] =(float)readData[0] - magBias_[0];
	data[1] =(float)readData[1] - magBias_[1];
	data[2] =(float)readData[2] - magBias_[2];
}

void MPU::getGyro(float* data) {
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
	int16_t mx, my, mz;
	mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
	data[0] = gx;
	data[1] = gy;
	data[2] = gz;
}
