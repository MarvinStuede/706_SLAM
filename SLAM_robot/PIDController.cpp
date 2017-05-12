/*
 * PIDController.cpp
 *
 *  Created on: May 10, 2017
 *      Author: marvin
 */

#include "PIDController.h"

PIDController::PIDController(float Kp, float Ki, float Kd,float stepSize) {
	Kp_ = Kp;
	Ki_ = Ki;
	Kd_ = Kd;
	errorOld_ = 0;
	Ta_ = 1/stepSize;
}

float PIDController::getControlVar(float error) {
	errorSum_ += error;
	float u = error*Kp_ + Ki_ * Ta_ * errorSum_ +Kd_/Ta_ * (error - errorOld_);
	errorOld_ = error;
	return u;
}

void PIDController::reset() {
	errorSum_ = 0;
}

