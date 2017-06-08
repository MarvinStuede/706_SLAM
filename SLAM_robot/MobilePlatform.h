/*
 * MobilePlatform.h
 *
 *  Created on: May 10, 2017
 *      Author: marvin
 */
#include <Servo.h>
#include <Arduino.h>
#include <LightChrono.h>
#include "PIDController.h"

#ifndef MOBILEPLATFORM_H_
#define MOBILEPLATFORM_H_

class MobilePlatform {
public:
	MobilePlatform();
	virtual ~MobilePlatform();
	void setup();
	void giveSensorVals(float* IRValues, float usDistance, float sfF, float sbF);
	void moveForward();
	void moveLeft();
	void moveBackward();
	void moveRight();
	void move();
	void turnLeft();
	void turnRight();
	void enableMotors();
	void disableMotors();
	void stop();
	void setSpeed(float vx, float vy, float omega);
	void setSpeed(float speed);
	bool isBatteryVoltageTooLow();
	bool keepWallDist(float distance, float& vx, float& vy, bool toSide = false);
	bool keepWallDist(float distance, float& vx, float& vy, float stepThreshold);
	bool keepWallAngle(float angle, float& omega, bool toSide = false);
	void setStepSize(float stepSize);
	float getIRAngle(bool side, bool filtered = false);
	float getIRMidDist(bool side);
	bool edgeDetected(float dt, float threshold);
	void resetDistSum();
	bool objectAvoidance(float thresholdFront, float thresholdSide, float& vx, float& vy, float& omega);
private:
	static const byte pinLeftFront_ = 46;
	static const byte pinLeftBack_ = 47;
	static const byte pinRightBack_ = 48;
	static const byte pinRightFront_ = 49;
	static constexpr float l1_ = 0.0863;
	static constexpr float l2_ = 0.0763;
	static constexpr float irDistSide_ = 0.145;
	static constexpr float Rw_ = 0.0275;
	float IRDistFrontLeft_ = 0;
	float IRDistFrontRight_ = 0;
	float IRDistSideFront_ = 0;
	float IRDistSideBack_ = 0;
	float IRDistSideFrontFiltered_ = 0;
	float IRDistSideBackFiltered_ = 0;
	float usDistFront_ = 0;
	Servo motorFrontLeft_;
	Servo motorFrontRight_;
	Servo motorBackLeft_;
	Servo motorBackRight_;
	LightChrono chronoBattery_;
	PIDController pidWallDist_;
	PIDController pidWallRot_;
	float speed_;
	float speedFrontLeft_;
	float speedFrontRight_;
	float speedBackLeft_;
	float speedBackRight_;
	float stepSize_;
	float vMax_ = 4.5;
	float omegaMax_ = 40;
	float distSum_ = 0;
	float AngleOld_ = 0;
	unsigned int distCnt_ = 1;
	inline void limit(float& val, float max)
	{
		val = val > max ? max : ((val < -max) ? -max : val);
	};

	int avoidanceState = 0; //Records the state of the object avoidance function;
	int nextState = 0;
	int stopCounter = 0;
	int detection = 0;
	void inverseKinematics(float &dt1, float &dt2, float &dt3, float &dt4, float vx, float vy, float omega);
};

#endif /* MOBILEPLATFORM_H_ */

