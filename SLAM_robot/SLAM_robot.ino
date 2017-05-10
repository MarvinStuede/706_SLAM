#include "Arduino.h"
#include "IRSensor.h"
#include "UltraSonicSensor.h"
#include "I2Cdev.h"
#include "MPU.h"
#include "Kalman.h"
#include "MobilePlatform.h"

MPU mpu;
//UltraSonicSensor ultrasonic;
IRSensor IR_front_left(SHARP_DX,A1);
IRSensor IR_front_right(SHARP_DX,A2);
IRSensor IR_side_front(SHARP_Ya,A3);
IRSensor IR_side_back(SHARP_Ya,A4);
MobilePlatform robot;

Kalman kalmanZ;
uint32_t timer;
float mag[3];

void setup()
{
	Serial.begin(115200);

	IR_front_left.setup(1.4197,-2.8392);
	IR_front_right.setup(1.1074,-0.4708);
	IR_side_front.setup(2.0813,-16.074);
	IR_side_back.setup(0.218,1.5159);
	ultrasonic.setup();
	mpu.setup();
	robot.setup();
	timer = micros();
}


void loop()
{
	double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
	float flDis,frDis,sfDis,sbDis;
	float calFl,calFr,calSf,calSb;
	timer = micros();
	//mpu.getGyro(mag);
	//Serial.print(mag[0]);
	//Serial.print(" ");
	//Serial.print(mag[1]);
	//Serial.print(" ");
	//Serial.print(mag[2]);
	//Serial.print(" ");
	//Serial.println();

	//		Serial.print(ultrasonic.getDistance());
	//		Serial.print(" ");
	//		Serial.println();

//	Serial.print("IR_fl: ");
//	calFl = IR_front_left.getValue(LINEAR);
//	Serial.print(calFl);
//	Serial.print(", IR_fr: ");
//	calFr = IR_front_right.getValue(LINEAR);
//	Serial.print(calFr);
//	Serial.print(", IR_sf: ");
//	calSf = IR_side_front.getValue(LINEAR);
//	Serial.print(calSf);
//	Serial.print(", IR_sb: ");
//	calSb = IR_side_back.getValue(EXPONENTIAL);
//	Serial.print(calSb);
//	Serial.println();


	delay(100);
}
