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

float PIDController::getControlVar(float error,float stepSize) {
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
}

