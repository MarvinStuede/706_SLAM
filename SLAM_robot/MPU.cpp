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

int MPU::readSensor(int addrL, int addrH) {
	  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
	  Wire.write(addrL);
	  Wire.endTransmission(false);

	  Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, true);
	  byte L = Wire.read();

	  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
	  Wire.write(addrH);
	  Wire.endTransmission(false);

	  Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, true);
	  byte H = Wire.read();

	  return (int16_t)((H<<8)+L);
}

int MPU::readSensor(int addr) {
	  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
	  Wire.write(addr);
	  Wire.endTransmission(false);

	  Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, true);
	  return Wire.read();
}

int MPU::writeSensor(int addr, int data) {
	  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
	  Wire.write(addr);
	  Wire.write(data);
	  Wire.endTransmission(true);

	  return 1;
}

void MPU::setupCompass() {
	  MPU9150_I2C_ADDRESS = 0x0C;      //change Address to Compass

	 writeSensor(0x0A, 0x00); //PowerDownMode
	 writeSensor(0x0A, 0x0F); //SelfTest
	 writeSensor(0x0A, 0x00); //PowerDownMode

	  MPU9150_I2C_ADDRESS = 0x68;      //change Address to MPU

	 writeSensor(0x24, 0x40); //Wait for Data at Slave0
	 writeSensor(0x25, 0x8C); //Set i2c address at slave0 at 0x0C
	 writeSensor(0x26, 0x02); //Set where reading at slave 0 starts
	 writeSensor(0x27, 0x88); //set offset at start reading and enable
	 writeSensor(0x28, 0x0C); //set i2c address at slv1 at 0x0C
	 writeSensor(0x29, 0x0A); //Set where reading at slave 1 starts
	 writeSensor(0x2A, 0x81); //Enable at set length to 1
	 writeSensor(0x64, 0x01); //overvride register
	 writeSensor(0x67, 0x03); //set delay rate
	 writeSensor(0x01, 0x80);

	 writeSensor(0x34, 0x04); //set i2c slv4 delay
	 writeSensor(0x64, 0x00); //override register
	 writeSensor(0x6A, 0x00); //clear usr setting
	 writeSensor(0x64, 0x01); //override register
	 writeSensor(0x6A, 0x20); //enable master i2c mode
	 writeSensor(0x34, 0x13); //disable slv4
}

void MPU::getGyro(int val[]) {
	val[0] = readSensor(MPU9150_GYRO_XOUT_L,MPU9150_GYRO_XOUT_H);
	val[1] = readSensor(MPU9150_GYRO_YOUT_L,MPU9150_GYRO_YOUT_H);
	val[2] = readSensor(MPU9150_GYRO_ZOUT_L,MPU9150_GYRO_ZOUT_H);
}

void MPU::setup() {
	 // Initialize the 'Wire' class for the I2C-bus.
	  Wire.begin();

	  // Clear the 'sleep' bit to start the sensor.
	  writeSensor(MPU9150_PWR_MGMT_1, 0);

	  setupCompass();
}
