/*
 * MPU.cpp
 *
 *  Created on: May 8, 2017
 *      Author: marvin
 */

#include "MPU.h"



MPU::MPU() {
	// TODO Auto-generated constructor stub
	magBias_[0] = 0;
	magBias_[1] = 0;
	magBias_[2] = 0;

}

MPU::~MPU() {
	// TODO Auto-generated destructor stub
}

void MPU::setup() {
	int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
	int myLed  = 13;  // Set up pin 13 led for toggling
	Wire.begin();

	// Set up the interrupt pin, its set as active high, push-pull
	pinMode(intPin, INPUT);
	digitalWrite(intPin, LOW);
	pinMode(myLed, OUTPUT);
	digitalWrite(myLed, HIGH);
	byte c = mpu.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
	Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
	Serial.print(" I should be "); Serial.println(0x71, HEX);

	if (c == 0x71) // WHO_AM_I should always be 0x68
	{
		Serial.println("MPU9250 is online...");

		// Start by performing self test and reporting values
		mpu.MPU9250SelfTest(mpu.SelfTest);
		Serial.print("x-axis self test: acceleration trim within : ");
		Serial.print(mpu.SelfTest[0],1); Serial.println("% of factory value");
		Serial.print("y-axis self test: acceleration trim within : ");
		Serial.print(mpu.SelfTest[1],1); Serial.println("% of factory value");
		Serial.print("z-axis self test: acceleration trim within : ");
		Serial.print(mpu.SelfTest[2],1); Serial.println("% of factory value");
		Serial.print("x-axis self test: gyration trim within : ");
		Serial.print(mpu.SelfTest[3],1); Serial.println("% of factory value");
		Serial.print("y-axis self test: gyration trim within : ");
		Serial.print(mpu.SelfTest[4],1); Serial.println("% of factory value");
		Serial.print("z-axis self test: gyration trim within : ");
		Serial.print(mpu.SelfTest[5],1); Serial.println("% of factory value");

		// Calibrate gyro and accelerometers, load biases in bias registers
		mpu.calibrateMPU9250(mpu.gyroBias, mpu.accelBias);

		mpu.initMPU9250();
		// Initialize device for active mode read of acclerometer, gyroscope, and
		// temperature
		Serial.println("MPU9250 initialized for active data mode....");

		// Read the WHO_AM_I register of the magnetometer, this is a good test of
		// communication
		byte d = mpu.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
		Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
		Serial.print(" I should be "); Serial.println(0x48, HEX);

		// Get magnetometer calibration from AK8963 ROM
		mpu.initAK8963(mpu.magCalibration);
		// Initialize device for active mode read of magnetometer
		Serial.println("AK8963 initialized for active data mode....");
		if (SerialDebug)
		{
			//  Serial.println("Calibration values: ");
			Serial.print("X-Axis sensitivity adjustment value ");
			Serial.println(mpu.magCalibration[0], 2);
			Serial.print("Y-Axis sensitivity adjustment value ");
			Serial.println(mpu.magCalibration[1], 2);
			Serial.print("Z-Axis sensitivity adjustment value ");
			Serial.println(mpu.magCalibration[2], 2);
		}
		//calibrate();
	}

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
		mpu.readMagData(mag_temp); // Read the mag data
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
	//mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
	magData[0] = mx;
	magData[1] = my;
	magData[2] = mz;
}

void MPU::getMag(float* data) {

	data[0] =(float)mpu.mx;
	data[1] =(float)mpu.my;
	data[2] =(float)mpu.mz;
}

void MPU::getGyro(float* data) {

	data[0] = (float)mpu.gx;
	data[1] = (float)mpu.gy;
	data[2] = (float)mpu.gz;
}

void MPU::readRegisters() {
	  if (mpu.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
	  {
	    mpu.readAccelData(mpu.accelCount);  // Read the x/y/z adc values
	    mpu.getAres();

	    // Now we'll calculate the accleration value into actual g's
	    // This depends on scale being set
	    mpu.ax = (float)mpu.accelCount[0]*mpu.aRes; // - accelBias[0];
	    mpu.ay = (float)mpu.accelCount[1]*mpu.aRes; // - accelBias[1];
	    mpu.az = (float)mpu.accelCount[2]*mpu.aRes; // - accelBias[2];

	    mpu.readGyroData(mpu.gyroCount);  // Read the x/y/z adc values
	    mpu.getGres();

	    // Calculate the gyro value into actual degrees per second
	    // This depends on scale being set
	    mpu.gx = (float)mpu.gyroCount[0]*mpu.gRes;
	    mpu.gy = (float)mpu.gyroCount[1]*mpu.gRes;
	    mpu.gz = (float)mpu.gyroCount[2]*mpu.gRes;

	    mpu.readMagData(mpu.magCount);  // Read the x/y/z adc values
	    mpu.getMres();
	    // User environmental x-axis correction in milliGauss, should be
	    // automatically calculated
	    mpu.magbias[0] = +470.;
	    // User environmental x-axis correction in milliGauss TODO axis??
	    mpu.magbias[1] = +120.;
	    // User environmental x-axis correction in milliGauss
	    mpu.magbias[2] = +125.;
//	    mpu.magbias[0] = magBias_[0];
//	    // User environmental x-axis correction in milliGauss TODO axis??
//	    mpu.magbias[1] = magBias_[1];
//	    // User environmental x-axis correction in milliGauss
//	    mpu.magbias[2] = magBias_[2];

	    // Calculate the magnetometer values in milliGauss
	    // Include factory calibration per data sheet and user environmental
	    // corrections
	    // Get actual magnetometer value, this depends on scale being set
	    mpu.mx = (float)mpu.magCount[0]*mpu.mRes*mpu.magCalibration[0] -
	               mpu.magbias[0];
	    mpu.my = (float)mpu.magCount[1]*mpu.mRes*mpu.magCalibration[1] -
	               mpu.magbias[1];
	    mpu.mz = (float)mpu.magCount[2]*mpu.mRes*mpu.magCalibration[2] -
	               mpu.magbias[2];
	  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

}

void MPU::updateQuaternions() {
	  mpu.updateTime();
	  // MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);

	  MahonyQuaternionUpdate(mpu.ax, mpu.ay, mpu.az, mpu.gx*DEG_TO_RAD,
	                         mpu.gy*DEG_TO_RAD, mpu.gz*DEG_TO_RAD, mpu.my,
	                         mpu.mx, mpu.mz, mpu.deltat);

}

void MPU::getRPY(float& roll, float& pitch, float& yaw) {
	updateQuaternions();

    mpu.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() *
                  *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1)
                  - *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
    mpu.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() *
                  *(getQ()+2)));
    mpu.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) *
                  *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1)
                  - *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));
    mpu.pitch *= RAD_TO_DEG;
    mpu.yaw   *= RAD_TO_DEG;
    mpu.yaw   -= 19.71;
    mpu.roll  *= RAD_TO_DEG;

    roll = mpu.roll;
    pitch = mpu.pitch;
    yaw = mpu.yaw;

}
