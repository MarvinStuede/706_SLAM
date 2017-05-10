/*
 * MobilePlatform.h
 *
 *  Created on: May 10, 2017
 *      Author: marvin
 */
#include <Servo.h>
#include <Arduino.h>
#include <LightChrono.h>

#ifndef MOBILEPLATFORM_H_
#define MOBILEPLATFORM_H_

class MobilePlatform {
public:
	MobilePlatform();
	virtual ~MobilePlatform();
	void setup();
	void moveForward();
	void moveLeft();
	void moveBackward();
	void moveRight();
	void turnLeft();
	void turnRight();
	void enableMotors();
	void disableMotors();
	void stop();
	void setSpeed(float speed);
	bool isBatteryVoltageTooLow();
private:
	static const byte pinLeftFront_ = 22;
	static const byte pinLeftBack_ = 23;
	static const byte pinRightBack_ = 24;
	static const byte pinRightFront_ = 25;
	static constexpr float l1_ = 0.0863;
	static constexpr float l2_ = 0.0763;
	static constexpr float Rw_ = 0.0275;
	Servo motorFrontLeft_;
	Servo motorFrontRight_;
	Servo motorBackLeft_;
	Servo motorBackRight_;
	LightChrono chronoBattery_;
	float speed_;

	void inverseKinematics(float &dt1,float &dt2,float &dt3,float &dt4,float vx, float vy, float omega);
};

#endif /* MOBILEPLATFORM_H_ */
