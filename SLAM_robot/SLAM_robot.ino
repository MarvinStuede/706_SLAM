#include "Arduino.h"
#include "IRSensor.h"
#include "UltraSonicSensor.h"
#include "I2Cdev.h"
#include "MPU.h"
#include "MobilePlatform.h"
#include "PIDController.h"
#include "LightChrono.h"
#include "Util.h"

MPU mpu;
UltraSonicSensor ultrasonic;
IRSensor IR_front_left(SHARP_DX,A1);
IRSensor IR_front_right(SHARP_DX,A2);
IRSensor IR_side_front(SHARP_YA,A3);
IRSensor IR_side_back(SHARP_YA,A4);
MobilePlatform robot;
LightChrono chrono_;
PIDController pidRotary(1.1,0.0000013,0.01);
PIDController pidSide(0.3, 0.0001, 0.001);

uint32_t timer;
float mag[3];
float gyr[3];
double dt = 0;
float stepSize = 10;

float angle = 0;
float kalAngle = 0;
double startTime = 0;
double prevTime = 0;
float angleDes = -90;
float distWall = 0;
float rotError = 0;
float IRValues[4];
float usDistance;
float ctrlVx = 0;
float ctrlVy = 0;
float ctrlOmega = 0;
float wallSpeed = 5.5;
double count = 0;
unsigned int wallDistIndex = 0;
unsigned int turnCnt = 0;
unsigned int distances = 4;
float stepThreshold = 8.0;
float wallDistances[4] = {10,25,40,55};
statesInit stateInit_ = INIT_SPIN;
statesMain stateMain_ = STATE_INIT;
//statesMain stateMain_ = STATE_DRIVE_WALL;
statesMain oldState_ = NONE;
float irAngle = 0;
float irAngleOld = 0;
float dAngle = 0;


void setup()
{
	prevTime = ((double)micros())/1000000;
	Serial.begin(115200);
	Serial1.begin(115200);

	IR_front_left.setup(1.4197,-2.8392);
	IR_front_right.setup(1.1074,-0.4708);
	IR_side_front.setup(1.4239,-3.4408);
	IR_side_back.setup(1.5945,-9.1103);
	ultrasonic.setup();
	mpu.setup();
	robot.setup();
	chrono_.start();
}

bool turnAngle(float angleGoal){
	ctrlOmega = pidRotary.getControlVar(angleGoal,angle,dt,0.075);
	if(pidRotary.isSettled(0.9)){
		pidRotary.reset();
		ctrlOmega = 0;
		return true;
	}
	else
		return false;
}
inline float rad2deg(float radVal){
	return radVal * 180/M_PI;
}
inline float deg2rad(float degVal){
	return degVal * M_PI/180;
}
void toState(statesMain state){
	oldState_ = stateMain_;
	stateMain_ = state;
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

		IRValues[0]= IR_front_left.getValue();
		IRValues[1] = IR_front_right.getValue();
		IRValues[2]= IR_side_front.getValue();
		IRValues[3] = IR_side_back.getValue();
		usDistance = ultrasonic.getDistance();
		robot.giveSensorVals(IRValues,usDistance);
		irAngle = robot.getIRAngle(true);
		dAngle = (irAngle - irAngleOld)/dt;
		irAngleOld = irAngle;
		robot.setStepSize(dt);

		switch(stateMain_){
		//INIT State: Referencing (drive to corner)
		case STATE_INIT:{
			switch(stateInit_){
			case INIT_SPIN:{
				ctrlOmega = 20;
				//Spin until wall to left found
				//Angle must be below the value and distance below max sensor values
				if(fabs(robot.getIRAngle(true)) < 5 && robot.getIRMidDist(true) < 110){
					stateInit_ = INIT_APP_WALL_1;
					ctrlOmega = 0;
				}
				break;
			}
			case INIT_APP_WALL_1:{
				//Approach wall to side
				if(robot.keepWallDist(wallDistances[0] + 20,ctrlVx,ctrlVy,true)){
					stateInit_ = INIT_APP_WALL_2;
					ctrlVy = 0;

				}

				break;
			}
			case INIT_APP_WALL_2:{
				//Align at wall
				if(robot.keepWallAngle(0,ctrlOmega,true))
					stateInit_ = INIT_APP_WALL_3;
				break;
			}
			case INIT_APP_WALL_3:{
				//Keep distance to wall and drive forwards
				robot.keepWallDist(wallDistances[0] + 20,ctrlVx,ctrlVy,true);
				robot.keepWallAngle(0,ctrlOmega,true);
				ctrlVx = wallSpeed;

				if(usDistance <= 10){
					ctrlVx = 0;
					ctrlVy = 0;
					ctrlOmega = 0;
					angleDes = -80;
					angle = 0;
					stateInit_ = INIT_APP_WALL_4;
				}
				break;
			}
			case INIT_APP_WALL_4:{
				//Turn so that robot faces away from wall
				if(turnAngle(angleDes)){
					toState(STATE_DRIVE_WALL);
					ctrlOmega = 0;
					ctrlVx = 0;
					ctrlVy = 0;
					angle = 0;
				}
				break;
			}
			}

			break;
		}
		//State: Drive along wall, keep distance and angle to wall
		case STATE_DRIVE_WALL:{

			robot.keepWallDist(wallDistances[wallDistIndex] + 20,ctrlVx,ctrlVy,true);
			//robot.keepWallAngle(0,ctrlOmega,true);
			ctrlOmega = pidRotary.getControlVar(0,angle,dt,0.1);
			//Speed along wall
			ctrlVx = wallSpeed;
			//Counter to check whether distance to wall must be increased
			//Stop if US sensor has defined distance to wall
			if(turnCnt == 3){
				//Still larger distances in array
				if(wallDistIndex + 1 <= distances -1){
					if	(usDistance <= wallDistances[wallDistIndex + 1]){
						turnCnt = 0;
						wallDistIndex++;
						ctrlVx = 0;
						ctrlVy = 0;
						ctrlOmega = 0;
						angleDes = -90;
						toState(STATE_TURN);
					}
				}
				//Reached center of field
				else if(wallDistIndex == distances -1){

					toState(STATE_WAIT);
				}
			}
			//Normal case, just turn
			else if(usDistance <= wallDistances[wallDistIndex]){
				ctrlVx = 0;
				ctrlVy = 0;
				ctrlOmega = 0;
				angleDes = -90;
				toState(STATE_TURN);
			}
			break;
		}
		//State: Turn for pre-defined angle
		case STATE_TURN:{
			if(oldState_ != stateMain_){
				oldState_ = stateMain_;
				angle = 0;
				robot.resetDistSum();
			}

			if(turnAngle(angleDes)){
				toState(STATE_DRIVE_WALL);
				turnCnt++;
				angle = 0;
				ctrlOmega = 0;
				ctrlVx = 0;
				ctrlVy = 0;
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
//		Serial.print(dAngle);
//		Serial.print(" ");
//		Serial.print(-30000);
//		Serial.print(" ");
//		Serial.print(30000);
//		Serial.print(" ");
//		Serial.println();
		robot.setSpeed(ctrlVx,ctrlVy,ctrlOmega);
		robot.move();
		//delay(10);

	}
	else
		Serial.println("VOLTAGE TOO LOW");
}








