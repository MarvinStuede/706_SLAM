/*
 * PIDController.h
 *
 *  Created on: May 10, 2017
 *      Author: marvin
 */

#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_
#include "Arduino.h"
class PIDController {
public:
	PIDController(float Kp, float Ki, float Kd);
	virtual ~PIDController();
	float getControlVar(float error,float stepSize);
	void reset();
private:
	float Kp_;
	float Ki_;
	float Kd_;
	float Ta_;
	float errorSum_;
	float errorOld_;
};

#endif /* PIDCONTROLLER_H_ */

