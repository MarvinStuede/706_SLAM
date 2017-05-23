#include "Arduino.h"
#include "IRSensor.h"
#include "UltraSonicSensor.h"
#include "I2Cdev.h"
#include "MPU.h"
#include "Kalman.h"
#include "MobilePlatform.h"
#include "PIDController.h"
#include "LightChrono.h"
#include "Util.h"

enum statesMain{
	STATE_INIT,
	STATE_TURN90,
	STATE_WAIT
}stateMain_ = STATE_INIT;

enum statesInit{
	INIT_SPIN,
	INIT_APP_WALL_1,
	INIT_APP_WALL_2
}stateInit_ = INIT_SPIN;

MPU mpu;
UltraSonicSensor ultrasonic;
IRSensor IR_front_left(SHARP_DX,A1);
IRSensor IR_front_right(SHARP_DX,A2);
IRSensor IR_side_front(SHARP_Ya,A3);
IRSensor IR_side_back(SHARP_Ya,A4);
MobilePlatform robot;
LightChrono chrono_;
PIDController pidRotary(0.6,0.0001,0);

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
float angleDes = 90;
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
	chrono_.start();
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

	IRValues[0]= IR_front_left.getValue(LINEAR);
	IRValues[1] = IR_front_right.getValue(LINEAR);
	IRValues[2]= IR_side_front.getValue(LINEAR);
	IRValues[3] = IR_side_back.getValue(LINEAR);
	robot.giveSensorVals(IRValues);
	robot.setStepSize(dt);

	switch(stateMain_){
	case STATE_INIT:{
		switch(stateInit_){
		case INIT_SPIN:{
			ctrlOmega = 60;
			//Spin until wall to left found
			if(rad2deg(robot.getIRAngle(true)) < 5 && robot.getIRMidDist(true) < 50){
				stateInit_ = INIT_APP_WALL_1;
				ctrlOmega = 0;
			}
			break;
		}
		case INIT_APP_WALL_1:{
			//Approach wall to side
			if(robot.approachWall(15,0.5,ctrlVx,ctrlVy,ctrlOmega,true))
				stateInit_ = INIT_APP_WALL_2;
			break;
		}
		case INIT_APP_WALL_2:{
				//Keep distance to wall and drive forwards
				robot.approachWall(15,0.5,ctrlVx,ctrlVy,ctrlOmega,true);
				ctrlVx = 2.5;
				if(ultrasonic.getDistance() <= 15){
					ctrlVx = 0;
					ctrlVy = 0;
					ctrlOmega = 0;
					stateMain_ = STATE_WAIT;
				}
			break;
		}
		}

		break;
	}
	case STATE_TURN90:{

		rotError = angleDes - angle;
		ctrlOmega = pidRotary.getControlVar(rotError,dt);
		if(fabs(rotError) < 5){
			stateMain_ = STATE_WAIT;
			ctrlOmega = 0;
			ctrlVx = 0;
			ctrlVy = 0;
			angleDes += 90;
			chrono_.restart();
		}
		break;
	}
	case STATE_WAIT:{
		ctrlVx = 0;
		ctrlVy = 0;
		ctrlOmega = 0;
		break;
	}
	}
//	Serial.print(ctrlVx);
//	Serial.print(" ");
	Serial.print(ctrlOmega);
	Serial.print(" ");
	Serial.println();
	robot.setSpeed(ctrlVx,ctrlVy,ctrlOmega);
	robot.move();


	delay(10);

}
else
	Serial.println("VOLTAGE TOO LOW");
}
bool turnAngle(float angleGoal, float threshold){
 	float e = angleGoal - angle;
 	ctrlOmega = pidRotary.getControlVar(e,dt);
 	if(fabs(e) < threshold){
 		ctrlOmega = 0;
 		return true;
 	}
 	else
 		return false;
 }
inline float rad2deg(float radVal){
	return radVal * 90/M_PI;
}
inline float deg2rad(float degVal){
	return degVal * M_PI/90;
}







