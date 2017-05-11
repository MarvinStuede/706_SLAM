/*
 * MobilePlatform.cpp
 *
 *  Created on: May 10, 2017
 *      Author: marvin
 */

#include "MobilePlatform.h"

MobilePlatform::MobilePlatform() {
	speed_ = 0;

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

void MobilePlatform::inverseKinematics(float& dt1, float& dt2, float& dt3,
		float& dt4, float vx, float vy, float omega) {
	dt1 = 1/Rw_ * (vx + vy - (l2_ + l1_) * omega);
	dt2 = 1/Rw_ * (vx - vy + (l2_ + l1_) * omega);
	dt3 = 1/Rw_ * (vx - vy - (l2_ + l1_) * omega);
	dt4 = 1/Rw_ * (vx + vy + (l2_ + l1_) * omega);

}

