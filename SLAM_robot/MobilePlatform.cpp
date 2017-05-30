/*
 * MobilePlatform.cpp
 *
 *  Created on: May 10, 2017
 *      Author: marvin
 */

#include "MobilePlatform.h"

MobilePlatform::MobilePlatform():
//pidWallDist_(0.5,0.0000001,0.01),
//pidWallRot_(0.415,0.000005,0.04){
pidWallDist_(1.5,0.0000,0),
pidWallRot_(1.8,0.0000004,0){
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

	pinMode(pinLeftFront_,INPUT);
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

	limit(vx,vMax_);
	limit(vy,vMax_);
	limit(omega,omegaMax_);

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

bool MobilePlatform::keepWallDist(float distance, float& vx, float& vy,
		bool toSide) {
	//Functions uses Controller to keep distance to wall

	if(toSide){
		vy =  pidWallDist_.getControlVar(distance,getIRMidDist(true),stepSize_);
	}
	else{
		vx = - pidWallDist_.getControlVar(distance,getIRMidDist(false),stepSize_);
	}
	if (pidWallDist_.isSettled(0.5)){
		pidWallDist_.reset();
		return true;
	}
	else{
		return false;
	}
}
bool MobilePlatform::keepWallDist(float distance, float& vx, float& vy,
		float stepThreshold) {
	float currentDist = getIRMidDist(true);
	if(distCnt_ == 1){
		distSum_ = currentDist;
		distCnt_++;
		vy =  pidWallDist_.getControlVar(distance,currentDist,stepSize_);
	}
	else{
		if(fabs(distSum_-distance) < stepThreshold){
			distSum_ += currentDist/distCnt_;
			vy =  pidWallDist_.getControlVar(distance,currentDist,stepSize_);
		}
	}

	if (pidWallDist_.isSettled(0.5)){
		pidWallDist_.reset();
		return true;
	}
	else{
		return false;
	}

}

bool MobilePlatform::keepWallAngle(float angle, float& omega, bool toSide) {
	//Function uses controller to keep angle to wall

	omega = -pidWallRot_.getControlVar(angle,getIRAngle(toSide),stepSize_,5);
	if (pidWallRot_.isSettled(0.3)){
		pidWallRot_.reset();
		return true;
	}
	else{

		return false;
	}
}

bool MobilePlatform::objectAvoidance(float thresholdFront, float thresholdSide, float& vx, float& vy, float& omega) {


	switch (avoidanceState)
	{
	case 0: {
		//Object is detected by front right sensor only
		if ((IRDistFrontLeft_ > thresholdFront) && (IRDistFrontRight_ < thresholdFront) && (usDistFront_ > thresholdFront-2)) {
			avoidanceState = 1;
		}else if ((IRDistFrontLeft_ > thresholdFront) && (IRDistFrontRight_ > thresholdFront) && (usDistFront_ < thresholdFront - 2)) {
			avoidanceState = 3;
		}
		else if ((IRDistFrontLeft_ < thresholdFront) && (IRDistFrontRight_ > thresholdFront) && (usDistFront_ > thresholdFront - 2)) {
			avoidanceState = 4;
		}

		return false; //Not necessary to carry out avoidance
	}
	case 1: {
		//stop the car
		vx = 0;
		vy = 0;
		omega = 0;
		avoidanceState = 2;
		break;
	}
	case 2: {
		//Move to Right until only sonar sensor detects
		if ((IRDistFrontLeft_ > thresholdFront) && (IRDistFrontRight_ > thresholdFront) && (usDistFront_ < thresholdFront-2)) {
			avoidanceState = 3;
		}
		vx = 0;
		vy = 3;
		omega = 0;
		break;
	}
	case 3: {
		//Move to the right until only right sensor detects
		if ((IRDistFrontLeft_ < thresholdFront) && (IRDistFrontRight_ > thresholdFront) && (usDistFront_ > thresholdFront-2)) {
			avoidanceState = 4;
		}
		vx = 0;
		vy = 3;
		omega = 0;
		break;
	}
	case 4: {
		//Move to the right until no sensor detects object
		if ((IRDistFrontLeft_ > thresholdFront) && (IRDistFrontRight_ > thresholdFront) && (usDistFront_ > thresholdFront-2)) {
			avoidanceState = 5;
		}
		vx = 0;
		vy = 3;
		omega = 0;
		break;
	}
	case 5: {
		//Move forward until front side sensor detects object
		if ((IRDistSideFront_ < thresholdSide) && (IRDistSideBack_ > thresholdSide)) {
			avoidanceState = 6;
		}
		vx = 3;
		vy = 0;
		omega = 0;
		break;
	}
	case 6: {
		//Move forward until back side sensor detects object
		if ((IRDistSideFront_ > thresholdSide) && (IRDistSideBack_ < thresholdSide)) {
			avoidanceState = 7;
		}
		vx = 3;
		vy = 0;
		omega = 0;
		break;
	}
	case 7: {
		//Move forward until no side sensor detects object
		if ((IRDistSideFront_ > thresholdSide) && (IRDistSideBack_ > thresholdSide)) {
			avoidanceState = 8;
		}
		vx = 3;
		vy = 0;
		omega = 0;
		break;
	}
	case 8: {
		vx = 0;
		vy = 0;
		omega = 0;
		avoidanceState = 0;
		break;
	}
	default:
		break;
	}


	return true; //obstacle avoidance is needed
}

void MobilePlatform::setStepSize(float stepSize) {
	stepSize_ = stepSize;
}
float MobilePlatform::getIRAngle(bool side) {
	if (side)
		return atan2((IRDistSideFront_-IRDistSideBack_)/100,irDistSide_) * 180/M_PI;
	else
		return atan2((IRDistFrontRight_-IRDistFrontLeft_)/100,2 * l1_) *180/M_PI;

}

void MobilePlatform::giveSensorVals(float* IRValues, float usDistance) {
	IRDistFrontLeft_ = IRValues[0];
	IRDistFrontRight_ = IRValues[1];
	IRDistSideFront_ = IRValues[2];
	IRDistSideBack_ = IRValues[3];
	usDistFront_ = usDistance;
}

float MobilePlatform::getIRMidDist(bool side) {
	if(side)
		return (IRDistSideFront_ + IRDistSideBack_)/2;
	else
		return (IRDistFrontRight_ + IRDistFrontLeft_)/2;
}

void MobilePlatform::resetDistSum() {
	distSum_ = 0;
	distCnt_ = 1;
}

void MobilePlatform::inverseKinematics(float& dt1, float& dt2, float& dt3,
		float& dt4, float vx, float vy, float omega) {
	dt1 = 1/Rw_ * (vx + vy - (l2_ + l1_) * omega);//Front left
	dt2 = 1/Rw_ * (vx - vy + (l2_ + l1_) * omega);//Front right
	dt3 = 1/Rw_ * (vx - vy - (l2_ + l1_) * omega);//Back left
	dt4 = 1/Rw_ * (vx + vy + (l2_ + l1_) * omega);//Back right

}

