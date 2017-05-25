/*
 * PIDController.cpp
 *
 *  Created on: May 10, 2017
 *      Author: marvin
 */

#include "PIDController.h"

PIDController::PIDController(float Kp, float Ki, float Kd) {
	Kp_ = Kp;
	Ki_ = Ki;
	Kd_ = Kd;
	errorOld_ = 0;
	errorSum_ = 0;
	Ta_ = 0;

}

float PIDController::getControlVar(float w, float r,float stepSize, float errorTol) {
	float error = w - r;

	//Timer to determine if controller is settled
	if(w/r > 0.9 && w/r < 1.1 ||
			(w == 0&& fabs(error) < errorTol)){
		if(!chronoSettled_.isRunning()){
			chronoSettled_.start();
		}
	}

	else if(chronoSettled_.isRunning()){
		chronoSettled_.restart();
		chronoSettled_.stop();
	}

	errorSum_ += error;
	Ta_ = 1/stepSize;
	float u = error * Kp_ + Ki_ * Ta_ * errorSum_ +Kd_/Ta_ * (error - errorOld_);
	errorOld_ = error;
	return u;
}

PIDController::~PIDController() {
}

void PIDController::reset() {
	errorSum_ = 0;
	errorOld_ = 0;
	chronoSettled_.restart();
	chronoSettled_.stop();
}

bool PIDController::isSettled(float timeThreshold) {
	//Returns true if value has been in +- 10% tolerance band of desired value
	//for specified time.

	return chronoSettled_.hasPassed(timeThreshold*1000);
}
