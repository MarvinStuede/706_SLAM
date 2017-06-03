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
	numValues = 0;
	corrParam1_ = 1;
	corrParam2_ = 0;
}

IRSensor::~IRSensor() {
	// TODO Auto-generated destructor stub
}

float IRSensor::getValue(bool useFilter) {
	float readVal;
	if(chrono_.elapsed() > 60)//Wait at least 60ms between measurements
	{
		if (useFilter)
			filteredVal_ = movingMedianFilter(readSensor(type_, pin_));
		readVal = readSensor(type_,pin_);

		oldDist_ = getCorrectLinear(readVal);

		chrono_.restart();
	}
	return oldDist_;
}

float IRSensor::movingMedianFilter(float curDistance){
	float smallest;
	int smallestIndex;
	int length = 4;
	float arraySort[length];

	if (numValues < length) {
		recordDistances[numValues] = curDistance;
		numValues++;
		return curDistance;
	}else {
		//shifts values up the array
		for (int i = 1; i < length; i++) {
			recordDistances[i - 1] = recordDistances[i];
			arraySort[i - 1] = recordDistances[i - 1];
		}

		recordDistances[length-1] = curDistance;
		arraySort[length-1] = curDistance;
	}


	for (int i = 0; i < length/2; i++) {
		smallest = arraySort[i];
		smallestIndex = i;
		for (int j = i+1; j < length; j++) {
			if (arraySort[j] < smallest) {
				smallest = arraySort[j];
				smallestIndex = j;
			}
		}

    if (i == 2) {
      return smallest;
    }

		if ((smallestIndex != i)&&(i != 4)) {
			arraySort[smallestIndex] = arraySort[i];
			arraySort[i] = smallest;
		}
	}

	return smallest;
}

float IRSensor::movingAverFilter(float curDistance){
	float sum = 0;
	if (numValues < 10) {
		recordDistances[numValues] = curDistance;
		numValues++;
	}else {
		for (int i = 1; i < 10; i++) {
			recordDistances[i-1] = recordDistances[i];
		}

		recordDistances[9] = curDistance;
	}

	for (int i = 0; i < numValues; i++) {
		sum += recordDistances[i];
	}

	return (float)sum/numValues;
}

void IRSensor::setup(float a1, float a2) {

	chrono_.start();
	corrParam1_ = a1;
	corrParam2_ = a2;
}

float IRSensor::readSensor(SHARP which_one, int which_analog_pin) {
	float temp_dis;
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
		if (temp_dis > 120.0)
			return 120.1;
		return temp_dis;
		break;
	}
}

float IRSensor::getCorrectLinear(float val) {
	return corrParam1_ * val + corrParam2_;
}

/*
float IRSensor::getCorrectExp(float val) {
	return corrParam1_ * pow(val,corrParam2_); **************DONT NEED THIS******************
}
*/
float IRSensor::getValueFiltered() {
	return filteredVal_;
}
