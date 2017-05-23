#include "Arduino.h"
#include "IRSensor.h"
#include "UltraSonicSensor.h"
#include "I2Cdev.h"
#include "MPU.h"
#include "Kalman.h"
#include "MobilePlatform.h"
#include "PIDController.h"

enum states{
	INIT,
	TURN90,
	WAIT
}state_;

MPU mpu;
UltraSonicSensor ultrasonic;
IRSensor IR_front_left(SHARP_DX,A1);
IRSensor IR_front_right(SHARP_DX,A2);
IRSensor IR_side_front(SHARP_YA,A3);
IRSensor IR_side_back(SHARP_YA,A4);
MobilePlatform robot;
PIDController pidRotary(0.6,0.00008,0);

Kalman kalmanZ;
uint32_t timer;
float mag[3];
float gyr[3];
double dt = 0;
float stepSize = 10;

//float FL[9];
//float FR[9];
//float SF[9];
//float SB[9];
float angle = 0;
float kalAngle = 0;
double startTime = 0;
double prevTime = 0;
float angleDes = 0;
float rotError = 0;
float IRValues[4];
float ctrlVx = 0;
float ctrlVy = 0;
float ctrlOmega = 0;


void setup()
{
	prevTime = ((double)micros())/1000000;
	Serial.begin(115200);

	IR_front_left.setup(1.4197,-2.8392);
	IR_front_right.setup(1.1074,-0.4708);
	IR_side_front.setup(1.4239,-3.4408);
	IR_side_back.setup(1.5945,-7.1103);
	ultrasonic.setup();
	mpu.setup();
	robot.setup();
	state_ = INIT;
}


void loop()
{
	if(!robot.isBatteryVoltageTooLow()){
	mpu.readRegisters(); //DO NOT DELETE
	dt = ((double)micros())/1000000 - prevTime;
	prevTime = ((double)micros())/1000000;

	//Read sensors
	mpu.getGyro(gyr);
	angle += gyr[2] * dt;
	rotError = angleDes - angle;

	//Read IR sensors
	IRValues[0]= IR_front_left.getValue(LINEAR);
	IRValues[1] = IR_front_right.getValue(LINEAR);
	IRValues[2]= IR_side_front.getValue(LINEAR);
	IRValues[3] = IR_side_back.getValue(LINEAR);
	robot.setStepSize(dt);

	switch(state_){
	case INIT:{
		state_ = WAIT;
		break;
	}
	case TURN90:{
		angleDes = 0;
		rotError = angleDes - angle;
		ctrlOmega = pidRotary.getControlVar(rotError,dt);
		if(fabs(rotError) < 3){
			state_ = WAIT;
			ctrlOmega = 0;
			ctrlVx = 0;
			ctrlVy = 0;
		}
		break;
	}
	case WAIT:{
		robot.approachWall(15,IRValues,0.5,ctrlVx,ctrlVy,ctrlOmega);
		ctrlVy = 2.5;
		break;
	}
	}
//	Serial.print(ctrlVx);
//	Serial.print(" ");
	//Serial.print(ctrlOmega);
	//Serial.print(" ");
	Serial.println();
	robot.setSpeed(ctrlVx,ctrlVy,ctrlOmega);
	robot.move();


	delay(10);

		//	Serial.print(", IR_fr: ");
	//	calFr = IR_front_right.getValue(LINEAR);
	//  Serial.print(calFr);
	//  Serial.print( IR_front_right.movingMedianFilter(calFr) );
	//  erial.print(medianFilter(calFr,2));

	//	Serial.print(", IR_sf: ");
	//	calSf = IR_side_front.getValue(LINEAR);
	//  Serial.print(calSf);
	//  Serial.print( IR_side_front.movingMedianFilter(calSf) );
	//	Serial.print(medianFilter(calSf,3));

	//	Serial.print(", IR_sb: ");
	//	calSb = IR_side_back.getValue(EXPONENTIAL);
	//  Serial.print(calSb);
	//  Serial.print( IR_side_back.movingMedianFilter(calSb) );
	//	Serial.print(medianFilter(calSb,4));
}
else
	Serial.println("VOLTAGE TOO LOW");
	
}

