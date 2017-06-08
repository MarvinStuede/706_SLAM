#include "Arduino.h"
#include "IRSensor.h"
#include "UltraSonicSensor.h"
#include "I2Cdev.h"
#include "MPU.h"
#include "MobilePlatform.h"
#include "PIDController.h"
#include "LightChrono.h"
#include "Chrono.h"
#include "Util.h"
#include "Mapping.h"

//car objects
MPU mpu;
UltraSonicSensor ultrasonic;
IRSensor IR_front_left(SHARP_DX, A1);
IRSensor IR_front_right(SHARP_DX, A2);
IRSensor IR_side_front(SHARP_YA, A3);
IRSensor IR_side_back(SHARP_YA, A4);
MobilePlatform robot;
//LightChrono chrono_;//, mapTimer_;
PIDController pidRotary(1.1, 0.0000013, 0.01);
//PIDController pidSide(0.3, 0.0001, 0.001);
Chrono chronoEdge;// , stopTime;
LightChrono chronoMap;
Mapping map_;

//Global variables
uint32_t timer;
float gyr[3];
double dt = 0;
float stepSize = 10;
float angle = 0;
double startTime = 0;
double prevTime = 0;
float angleDes = -85;
float distWall = 0;
float rotError = 0;
float IRValues[4];
float usDistance;
float ctrlVx = 0;
float ctrlVy = 0;
float ctrlOmega = 0;
float wallSpeed = 5;
unsigned int wallDistIndex = 0;
unsigned int turnCnt = 0;
unsigned int distances = 4;
float stepThreshold = 8.0;
float wallDistances[4] = { 10,25,40,55 };
statesInit stateInit_ = INIT_SPIN;
statesMain stateMain_ = STATE_INIT;
statesTurn stateTurn = TURN_GYRO;
//statesMain stateMain_ = STATE_DRIVE_WALL;
statesMain oldState_ = NONE;
float irAngle = 0;
float irAngleOld = 0;
float dAngle = 0;
int trig = 0;
float sfF = 0;
float sbF = 0;
float sideDISTANCE = 0;
bool measuredSide = false;
int stopCount = 0;

void setup()
{
	prevTime = ((double)micros()) / 1000000;
	Serial.begin(115200);
	Serial1.begin(115200);

	IR_front_left.setup(1.4197, -4.8392);//-2.8392
	IR_front_right.setup(1.1074, -2.4708); //0.4708
	IR_side_front.setup(1.5239, -3.4408); //-3.4408
	IR_side_back.setup(1.4945, -4.1103); //-9.1103
	ultrasonic.setup();
	mpu.setup();
	robot.setup();
	//chrono_.start();
}

bool turnAngle(float angleGoal) {
	ctrlOmega = pidRotary.getControlVar(angleGoal, angle, dt, 0.1);

	if (pidRotary.isSettled(0.5)) {
		pidRotary.reset();
		ctrlOmega = 0;
		return true;
	}
	else
		return false;
}

bool noObjectToSide(float dt, float threshold) {
	if (chronoEdge.isRunning()) {
		if (chronoEdge.elapsed() > 1500) {
			chronoEdge.restart();
			chronoEdge.stop();
		}
		return false;
	}
	else if (robot.edgeDetected(dt, threshold)) {
		chronoEdge.start();
		return false;
	}
	else
		return true;
}

inline float rad2deg(float radVal) {
	return radVal * 180 / M_PI;
}
inline float deg2rad(float degVal) {
	return degVal * M_PI / 180;
}
void toState(statesMain state) {
	oldState_ = stateMain_;
	stateMain_ = state;
}

void loop()
{
	if (!robot.isBatteryVoltageTooLow()) {
		mpu.readRegisters();	//Read the mpu data
		dt = ((double)micros()) / 1000000 - prevTime; //Get the change in time from previous loop
		prevTime = ((double)micros()) / 1000000;

		mpu.getGyro(gyr);	//Get gyro data
		angle += gyr[2] * dt; //calculate yaw angle of car
		robot.setStepSize(dt);
		//rotError = angleDes - angle;

		//Get IR readings
		IRValues[0] = IR_front_left.getValue(false);
		IRValues[1] = IR_front_right.getValue(false);
		IRValues[2] = IR_side_front.getValue(true);
		IRValues[3] = IR_side_back.getValue(true);

		usDistance = ultrasonic.getDistance();	//Get ultrasonic reading
		//IRValues[0] = IR_front_left.getValueFiltered();
		//IRValues[1] = IR_front_right.getValueFiltered();

		sfF = IR_side_front.getValueFiltered();
		sbF = IR_side_back.getValueFiltered();

		//sfF = IRValues[2];
		//sbF = IRValues[3];

		robot.giveSensorVals(IRValues, usDistance, sfF, sbF);
		//irAngle = robot.getIRAngle(true, false);
		//dAngle = (irAngle - irAngleOld) / dt;
		//irAngleOld = irAngle;

		switch (stateMain_) {
			//INIT State: Referencing (drive to corner)
		case STATE_INIT: {
			switch (stateInit_) {
			case INIT_SPIN: {
				ctrlOmega = 20;
				//Spin until wall to left found
				//Angle must be below the value and distance below max sensor values
				if ((fabs(robot.getIRAngle(true, true)) < 5) && robot.getIRMidDist(true) < 110) {
					stateInit_ = INIT_APP_WALL_1;
					ctrlOmega = 0;
				}
				break;
			}
			case INIT_APP_WALL_1: {
				//Approach wall to side
				if (robot.keepWallDist(wallDistances[0] + 20, ctrlVx, ctrlVy, true)) {
					stateInit_ = INIT_APP_WALL_2;
					ctrlVy = 0;
					ctrlVx = 0;
				}

				break;
			}
			case INIT_APP_WALL_2: {
				//Align at wall
				if (robot.keepWallAngle(0, ctrlOmega, true)) {
					stateInit_ = INIT_APP_WALL_3;
					angle = 0;
					angleDes = 0;
				}
				break;
			}
			case INIT_APP_WALL_3: {
				if (robot.objectAvoidance(10, 26, ctrlVx, ctrlVy, ctrlOmega)) {
					toState(STATE_OBSTACLE);
				}
				else if (usDistance < 10) {
					ctrlVx = 0;
					ctrlVy = 0;
					ctrlOmega = 0;
					angleDes = -90;
					angle = 0;
					stateInit_ = INIT_APP_WALL_4;
				}
				else{

					//Keep distance to wall and drive forwards
					robot.keepWallDist(wallDistances[0] + 20, ctrlVx, ctrlVy, true);
				robot.keepWallAngle(0, ctrlOmega, true);
				//turnAngle(angleDes);
				ctrlVx = wallSpeed;
			}
				break;
			}
			case INIT_APP_WALL_4: {
				//Turn so that robot faces away from wall
				if (turnAngle(angleDes)) {
					toState(STATE_DRIVE_WALL);
					ctrlOmega = 0;
					ctrlVx = 0;
					ctrlVy = 0;
					angle = 0;

					//Initialise map: origin
chronoMap.start();
					map_.initialise(usDistance, robot.getIRMidDist(true));
				}
				break;
			}
			}

			break;
		}
						 //State: Drive along wall, keep distance and angle to wall
		case STATE_DRIVE_WALL: {
			//Check for object avoidance
			if (robot.objectAvoidance(10, 26, ctrlVx, ctrlVy, ctrlOmega)) {
				toState(STATE_OBSTACLE);
			}
			else if (noObjectToSide(dt, 5700)) {
				robot.keepWallDist(wallDistances[wallDistIndex] + 20, ctrlVx, ctrlVy, true);
				robot.keepWallAngle(0, ctrlOmega, true);
				//ctrlOmega = pidRotary.getControlVar(0, angle, dt, 1);
				//trig = 0;
			}
			else
				trig = 1;
		
			//ctrlOmega = pidRotary.getControlVar(0, angle, dt, 1);

			//Speed along wall
			ctrlVx = wallSpeed;

			//ctrlVy =  0;

			//Counter to check whether distance to wall must be increased
			//Stop if US sensor has defined distance to wall
			if (turnCnt == 3) {
				//Still larger distances in array
				if (wallDistIndex + 1 <= distances - 1) {
					if ((usDistance <= wallDistances[wallDistIndex + 1])) {// && (sbF < wallDistances[wallDistIndex + 1] + 4)&& (sfF < wallDistances[wallDistIndex + 1] + 4)){
					//if (fabs((IRValues[0] + IRValues[1] + usDistance + 4)/3 - wallDistances[wallDistIndex + 1] ) <= 5) {
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
				else if (wallDistIndex == distances - 1) {

					toState(STATE_WAIT);
				}
			}
			//Normal case, just turn
			else if (usDistance <= wallDistances[wallDistIndex]) {
				//else if (fabs((IRValues[0] + IRValues[1] + usDistance + 4)/3 - wallDistances[wallDistIndex]) <= 5){
					ctrlVx = 0;
					ctrlVy = 0;
					ctrlOmega = 0;
					angleDes = -90;
					toState(STATE_TURN);

				toState(STATE_TURN);
			}

			break;
		}
							   //State: Turn for pre-defined angle
		case STATE_TURN: {
			if (oldState_ != stateMain_) {
				oldState_ = stateMain_;
				angle = 0;
				robot.resetDistSum();
			}
			switch (stateTurn) {
			case TURN_GYRO: {
				if (turnAngle(angleDes)) {
					stateTurn = TURN_WALL;
				}
				break;
			}
			case TURN_WALL: {
				if (robot.keepWallAngle(0, ctrlOmega, true)|| turnAngle(angleDes)) {
					toState(STATE_DRIVE_WALL);
					turnCnt++;
					map_.robotTurned(usDistance,robot.getIRMidDist(true),turnCnt+1);
					angle = 0;
					ctrlOmega = 0;
					ctrlVx = 0;
					ctrlVy = 0;
					stateTurn = TURN_GYRO;
				}
				break;
			}
			}
			break;
		}
		case STATE_WAIT: {
			ctrlVx = 0;
			ctrlVy = 0;
			ctrlOmega = 0;
			break;
		}
		case STATE_OBSTACLE: {
			if (!robot.objectAvoidance(10, 26, ctrlVx, ctrlVy, ctrlOmega)) {
				toState(oldState_);
			}

			//robot.keepWallAngle(0, ctrlOmega, true); //make sure the car keeps constant yaw angle while avoiding obstacle
			break;
		}
		case STATE_MOVE_LEFT: {

			//record side distance
			if (!measuredSide){//stopTime.isRunning() || !measuredSide) {
				//measuredSide = true;
				sideDISTANCE = robot.getIRMidDist(true);
				stopCount++;

				if (stopCount > 100) {
					measuredSide = true;
					stopCount = 0;
				}
				else {
					ctrlVx = 0;
					ctrlVy = 0;
					ctrlOmega = 0;
					break;
				}
				
			}

			

			//Object or wall is detected by sonar. Move left to stop sonar detecting object else it is a wall.
			ctrlVx = 0;
			ctrlVy = 4;
			ctrlOmega = 0;

			
			if (robot.objectAvoidance(10, 26, ctrlVx, ctrlVy, ctrlOmega)) {
				toState(STATE_OBSTACLE);
			}

			//Sonar has detected an object, once it detects the wall, after moving left, again return to moving straight.
			else if (usDistance > wallDistances[wallDistIndex]) {
				measuredSide = false;
				toState(STATE_DRIVE_WALL);
			}
			//Moved left for 5cm and still seeing object, it must be a wall.
			else if (robot.getIRMidDist(true) > sideDISTANCE + 5) {
				ctrlVx = 0;
				ctrlVy = 0;
				ctrlOmega = 0;
				angleDes = -90;
				measuredSide = false;
				toState(STATE_TURN);
			}
		}
		}

		/*if (trig == 1)
			Serial.print(50);
		else
			Serial.print(0);
		Serial.print(" ");
		//		Serial.print(-5000);
		//		Serial.print(" ");
		//		Serial.print(5000);
		//		Serial.print(" ");
		Serial.println();
		*/

		//Controls motors for moving
		robot.setSpeed(ctrlVx, ctrlVy, ctrlOmega);
		robot.move();

		/*
		Serial.print((IRValues[0] + IRValues[1] + usDistance + 4) / 3);
		Serial.print(" ");
		
		Serial.print(sbF);
		Serial.print(" ");
		Serial.print(usDistance);
		Serial.print(" ");
		Serial.println(sfF);
		*/
		/*Serial1.println(255);
		Serial.print(robot.getIRAngle(true, true));
		Serial.print(" ");
		Serial.print(sbF);
		Serial.print(" ");
		Serial.println(sfF);
		*/
		if(chronoMap.elapsed() > 250){
			chronoMap.restart();
			map_.savePoint(usDistance,robot.getIRMidDist(true),turnCnt+1);
			map_.sendCurrentPoint(turnCnt);
		}
	}
	else
		Serial.println("VOLTAGE TOO LOW");
}