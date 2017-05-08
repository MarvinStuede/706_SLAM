#include "Arduino.h"
#include "libraries/MPU/MPU.h"
#include "libraries/IRSensor/IRSensor.h"

MPU MPU9050;
IRSensor IR_front_left(SHARP_DX,A4);
IRSensor IR_front_right(SHARP_DX,A6);
IRSensor IR_side_front(SHARP_Ya,A5);
IRSensor IR_side_back(SHARP_Ya,A7);

int gyro[3];

void setup()
{
	Serial.begin(115200);
	MPU9050.setup();
}


void loop()
{
	MPU9050.getGyro(gyro);

//	Serial.print("G_x: ");
//	Serial.print(gyro[0]);
//	Serial.print(", G_y: ");
//	Serial.print(gyro[1]);
//	Serial.print(", G_z: ");
//	Serial.print(gyro[2]);
//	Serial.println();
	Serial.print("IR_fl: ");
	Serial.print(IR_front_left.getVal());
	Serial.print(", IR_fr: ");
	Serial.print(IR_front_right.getVal());
	Serial.print(", IR_sf: ");
	Serial.print(IR_side_front.getVal());
	Serial.print(", IR_sb: ");
	Serial.print(IR_side_back.getVal());
	Serial.println();

	delay(100);
}
