/*
 * IRSensor.cpp
 *
 *  Created on: May 8, 2017
 *      Author: marvin
 */

#include "IRSensor.h"


IRSensor::IRSensor(SHARP type, int pin) {
	type_ = type;
	pin_ = pin;
	oldDist_ = 0;
}

IRSensor::~IRSensor() {
	// TODO Auto-generated destructor stub
}

int IRSensor::getVal() {
	if(chrono_.elapsed() > 60)//Wait at least 60ms between measurements
	{
		oldDist_ = read(type_,pin_);
		chrono_.restart();
	}
	return oldDist_;
}

void IRSensor::setup() {
	chrono_.start();
}

int IRSensor::read(SHARP which_one, int which_analog_pin) {
	int temp_dis;
	switch (which_one) {
	case SHARP_DX:
		//2D120X 4cm - 30cm http://www.phidgets.com/products.php?product_id=3520
		temp_dis = 2076.0 / (analogRead(which_analog_pin) - 11) + 2;
		if (temp_dis < 0.0)
			return 30.1;
		if (temp_dis >= 0.0 && temp_dis <= 4.0)
			return 3.9;
		if (temp_dis > 30.0)
			return 30.1;
		return temp_dis;
		break;
	case SHARP_Ya:
		//2Y0A21 10cm - 80cm http://www.phidgets.com/products.php?product_id=3521
		temp_dis =  4800.0 / (analogRead(which_analog_pin) - 20);
		if (temp_dis < 0.0)
			return 80.1;
		if (temp_dis >= 0.0 && temp_dis <= 10.0)
			return 9.9;
		if (temp_dis > 80.0)
			return 80.1;
		return temp_dis;
		break;
	case SHARP_YA:
		//2Y0A02 20cm - 150cm http://www.phidgets.com/products.php?product_id=3522
		temp_dis =  9462.0 / (analogRead(which_analog_pin) - 16.92);
		if (temp_dis < 0.0)
			return 150.1;
		if (temp_dis >= 0.0 && temp_dis <= 20.0)
			return 19.9;
		if (temp_dis > 150.0)
			return 150.1;
		return temp_dis;
		break;
	}
}
