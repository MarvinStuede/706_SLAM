/*
 * PIDController.h
 *
 *  Created on: May 10, 2017
 *      Author: marvin
 */

#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_
#include "Arduino.h"
#include "Chrono.h"
class PIDController {
public:
	PIDController(float Kp, float Ki, float Kd);
	virtual ~PIDController();
	float getControlVar(float w, float r,float stepSize, float errorTol = 0);
	void reset();
	bool isSettled(float timeThreshold);
private:
	float Kp_;
	float Ki_;
	float Kd_;
	float Ta_;
	float errorSum_;
	float errorOld_;
	Chrono chronoSettled_;
};

#endif /* PIDCONTROLLER_H_ */

