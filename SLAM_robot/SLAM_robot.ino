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

		Serial.print("IR_fl: ");
    flDis = IR_front_left.getVal();
    calFl = 1.4197 * flDis - 2.8392;
		Serial.print(calFl);
		Serial.print(", IR_fr: ");
    frDis = IR_front_right.getVal();
    calFr = 1.1074 * frDis - 0.4708;
		Serial.print(calFr);
		Serial.print(", IR_sf: ");
    sfDis = IR_side_front.getVal();
    calSf = 2.0813 * sfDis - 16.074;
		Serial.print(calSf);
		Serial.print(", IR_sb: ");
    sbDis = IR_side_back.getVal();
    calSb = 1 * sbDis - 0; 
		Serial.print(calSb);
		Serial.println();

	delay(100);
}
