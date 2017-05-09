#include "Arduino.h"
#include "IRSensor.h"
#include "UltraSonicSensor.h"
#include "I2Cdev.h"
#include "MPU.h"
#include "Kalman.h"

MPU mpu;
UltraSonicSensor ultrasonic;
IRSensor IR_front_left(SHARP_DX,A1);
IRSensor IR_front_right(SHARP_DX,A2);
IRSensor IR_side_front(SHARP_Ya,A3);
IRSensor IR_side_back(SHARP_Ya,A4);
Kalman kalmanZ;
uint32_t timer;
float mag[3];

void setup()
{
	Serial.begin(115200);
	IR_front_left.setup();
	IR_front_right.setup();
	IR_side_front.setup();
	IR_side_back.setup();
	ultrasonic.setup();
	mpu.setup();
	timer = micros();
}


void loop()
{
	double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
	timer = micros();
	mpu.getGyro(mag);
	Serial.print(mag[0]);
	Serial.print(" ");
	Serial.print(mag[1]);
	Serial.print(" ");
	Serial.print(mag[2]);
	Serial.print(" ");
	Serial.println();

		//Serial.print(ultrasonic.getDistance());
//		Serial.print(" ");
//		Serial.println();

//		Serial.print("IR_fl: ");
//		Serial.print(IR_front_left.getVal());
//		Serial.print(", IR_fr: ");
//		Serial.print(IR_front_right.getVal());
//		Serial.print(", IR_sf: ");
//		Serial.print(IR_side_front.getVal());
//		Serial.print(", IR_sb: ");
//		Serial.print(IR_side_back.getVal());
//		Serial.println();

	delay(100);
}
