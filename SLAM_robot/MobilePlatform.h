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
	static const byte pinLeftFront_ = 46;
	static const byte pinLeftBack_ = 47;
	static const byte pinRightBack_ = 48;
	static const byte pinRightFront_ = 49;
	Servo motorFrontLeft_;
	Servo motorFrontRight_;
	Servo motorBackLeft_;
	Servo motorBackRight_;
	LightChrono chronoBattery_;
	float speed_;
};

#endif /* MOBILEPLATFORM_H_ */
