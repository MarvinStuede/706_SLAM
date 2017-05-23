/*
 * MobilePlatform.cpp
 *
 *  Created on: May 10, 2017
 *      Author: marvin
 */

#include "MobilePlatform.h"

MobilePlatform::MobilePlatform():
pidWallDist_(0.5,0.0001,0),
pidWallRot_(5,0,0){
	speed_ = 0;
	speedFrontLeft_ = 0;
	speedFrontRight_ = 0;
	speedBackLeft_ = 0;
	speedBackRight_ = 0;
	stepSize_ = 0;

	avoidanceState = 0; //No obstacle avoidance required at beginning (state 0);
}

MobilePlatform::~MobilePlatform() {
	// TODO Auto-generated destructor stub
}

void MobilePlatform::moveForward() {
	  motorFrontLeft_.writeMicroseconds(1500 + speed_);
	  motorBackLeft_.writeMicroseconds(1500 + speed_);
	  motorBackRight_.writeMicroseconds(1500 - speed_);
	  motorFrontRight_.writeMicroseconds(1500 - speed_);
}

void MobilePlatform::moveLeft() {
	  motorFrontLeft_.writeMicroseconds(1500 - speed_);
	  motorBackLeft_.writeMicroseconds(1500 + speed_);
	  motorBackRight_.writeMicroseconds(1500 + speed_);
	  motorFrontRight_.writeMicroseconds(1500 - speed_);
}

void MobilePlatform::moveBackward() {
	  motorFrontLeft_.writeMicroseconds(1500 - speed_);
	  motorBackLeft_.writeMicroseconds(1500 - speed_);
	  motorBackRight_.writeMicroseconds(1500 + speed_);
	  motorFrontRight_.writeMicroseconds(1500 + speed_);
}

void MobilePlatform::moveRight() {
	  motorFrontLeft_.writeMicroseconds(1500 + speed_);
	  motorBackLeft_.writeMicroseconds(1500 - speed_);
	  motorBackRight_.writeMicroseconds(1500 - speed_);
	  motorFrontRight_.writeMicroseconds(1500 + speed_);
}

void MobilePlatform::move() {
	  motorFrontLeft_.writeMicroseconds(1500 + speedFrontLeft_);
	  motorBackLeft_.writeMicroseconds(1500 + speedBackLeft_);
	  motorBackRight_.writeMicroseconds(1500 - speedBackRight_);
	  motorFrontRight_.writeMicroseconds(1500 - speedFrontRight_);
}

void MobilePlatform::turnLeft() {
	  motorFrontLeft_.writeMicroseconds(1500 - speed_);
	  motorBackLeft_.writeMicroseconds(1500 - speed_);
	  motorBackRight_.writeMicroseconds(1500 - speed_);
	  motorFrontRight_.writeMicroseconds(1500 - speed_);
}

void MobilePlatform::turnRight() {
	  motorFrontLeft_.writeMicroseconds(1500 + speed_);
	  motorBackLeft_.writeMicroseconds(1500 + speed_);
	  motorBackRight_.writeMicroseconds(1500 + speed_);
	  motorFrontRight_.writeMicroseconds(1500 + speed_);
}

void MobilePlatform::enableMotors() {
	motorFrontLeft_.attach(pinLeftFront_);  // attaches the servo on pin left_front to the servo object
	motorBackLeft_.attach(pinLeftBack_);  // attaches the servo on pin left_rear to the servo object
	motorBackRight_.attach(pinRightBack_);  // attaches the servo on pin right_rear to the servo object
	motorFrontRight_.attach(pinRightFront_);  // attaches the servo on pin right_front to the servo object
}

void MobilePlatform::disableMotors() {
	motorFrontLeft_.detach();  // detach the servo on pin left_front to the servo object
	motorBackLeft_.detach();  // detach the servo on pin left_rear to the servo object
	motorBackRight_.detach();  // detach the servo on pin right_rear to the servo object
	motorFrontRight_.detach();  // detach the servo on pin right_front to the servo object

	pinMode(pinLeftFront_, INPUT);
	pinMode(pinLeftBack_, INPUT);
	pinMode(pinRightBack_, INPUT);
	pinMode(pinRightFront_, INPUT);
}

void MobilePlatform::stop() {
	  motorFrontLeft_.writeMicroseconds(1500);
	  motorBackLeft_.writeMicroseconds(1500);
	  motorBackRight_.writeMicroseconds(1500);
	  motorFrontRight_.writeMicroseconds(1500);
}

void MobilePlatform::setSpeed(float vx, float vy, float omega) {
	inverseKinematics(speedFrontLeft_,speedFrontRight_,speedBackLeft_,speedBackRight_,vx,vy,omega);

}
void MobilePlatform::setSpeed(float speed) {
	speed_ = speed;
}

void MobilePlatform::setup() {
	enableMotors();
	chronoBattery_.start();
}

bool MobilePlatform::isBatteryVoltageTooLow() {

	  if (chronoBattery_.elapsed() > 500) { //500ms timed if statement to check lipo and output speed settings
	    int Lipo_level_cal;
	    //the voltage of a LiPo cell depends on its chemistry and varies from about 2.7-3.1 V (discharged) = 620(3.1V Min)
	    //to about 4.20-4.25 V (fully charged) = 820(4.1V Max)
	    Lipo_level_cal = (analogRead(A0) - 620);
	    Lipo_level_cal = Lipo_level_cal * 100;
	    Lipo_level_cal = Lipo_level_cal / 200;
	    chronoBattery_.restart();
	    if (Lipo_level_cal < 0) return false;
	  }
	  return false;
}

bool MobilePlatform::approachWall(float distance, float threshold, float& vx, float& vy, float& omega,
		bool toSide) {
	float error_dist = 0;
	float error_rot = 0;
	if(toSide){
		error_dist = getIRMidDist(true) - distance;
		error_rot = getIRAngle(true) * 90/M_PI;
		vy = - pidWallDist_.getControlVar(error_dist,stepSize_);
		omega = pidWallRot_.getControlVar(error_rot,stepSize_);
	}
	else{

	error_dist = getIRMidDist(false) - distance;
	error_rot = getIRAngle(false) * 90/M_PI;



	if (fabs(error_dist/distance) < threshold && fabs(error_rot) < threshold && false){
		pidWallDist_.reset();
		pidWallRot_.reset();
		return true;
	}
	else{
		vx = pidWallDist_.getControlVar(error_dist,stepSize_);
		omega = pidWallRot_.getControlVar(error_rot,stepSize_);
		return false;
	}
	}
}
/*bool MobilePlatform::objectAvoidance(float* IRvalues, float threshold, float& vx, float& vy, float& omega) {
	//An object is in front of the right IR sensor or the object avoidance was previously running
	if ((IRValues[1] < threshold)) {
		avoidanceState = 1;
	}

	switch (avoidanceState)
	{
	case 1: {
		//stop the car
		vx = 0;
		vy = 0;
	}
	break;
	default:
		break;
	}

	if (avoidanceState == 0) {
		return false; //Not necessary to carry out avoidance
	}
	else {
		return true; //obstacle avoidance is needed
	}
}
*/
void MobilePlatform::setStepSize(float stepSize) {
	stepSize_ = stepSize;
}
float MobilePlatform::getIRAngle(bool side) {
	if(side)
		return atan2((IRDistSideFront_-IRDistSideBack_)/100,irDistSide_) * 90/M_PI;
	else
		return atan2((IRDistFrontRight_-IRDistFrontLeft_)/100,2 * l1_) * 90/M_PI;
}

void MobilePlatform::giveSensorVals(float* IRValues) {
	IRDistFrontLeft_ = IRValues[0];
	IRDistFrontRight_ = IRValues[1];
	IRDistSideFront_ = IRValues[2];
	IRDistSideBack_ = IRValues[3];
}

float MobilePlatform::getIRMidDist(bool side) {
	if(side)
		return (IRDistSideFront_ + IRDistSideBack_)/2;
	else
		return (IRDistFrontRight_ + IRDistFrontLeft_)/2;
}
void MobilePlatform::inverseKinematics(float& dt1, float& dt2, float& dt3,
		float& dt4, float vx, float vy, float omega) {
	dt1 = 1/Rw_ * (vx + vy - (l2_ + l1_) * omega);//Front left
	dt2 = 1/Rw_ * (vx - vy + (l2_ + l1_) * omega);//Front right
	dt3 = 1/Rw_ * (vx - vy - (l2_ + l1_) * omega);//Back left
	dt4 = 1/Rw_ * (vx + vy + (l2_ + l1_) * omega);//Back right

}

