#include "Arduino.h"
#include "libraries/MPU/MPU.h"
MPU MPU9050;
int gyro[3];

void setup()
{
	Serial.begin(9600);
	MPU9050.setup();
}


void loop()
{
	MPU9050.getGyro(gyro);

	Serial.print(gyro[0]);
	Serial.print("  ");
	Serial.print(gyro[1]);
	Serial.print("  ");
	Serial.print(gyro[2]);
	Serial.print("  ");
	Serial.println();

	delay(100);
}
