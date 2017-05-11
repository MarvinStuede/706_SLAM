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

float IRSensor::getValue(corrType type) {
	float readVal;
	if(chrono_.elapsed() > 60)//Wait at least 60ms between measurements
	{
		readVal = movingMedianFilter(read(type_, pin_));
    readVal = read(type_, pin_);
		if(type == LINEAR){
			oldDist_ = getCorrectLinear(readVal);
		}
		else if(type == EXPONENTIAL){
			oldDist_ = getCorrectExp(readVal);
		}

		chrono_.restart();
	}
	return oldDist_;
}

float IRSensor::movingMedianFilter(float curDistance){
  float sum = 0;
  float holder = 0;
  if (numValues < 9) {
    recordDistances[numValues] = curDistance;
    numValues++;
  }else {
    for (int i = 1; i < 9; i++) {
      recordDistances[i-1] = recordDistances[i];
    }

    recordDistances[8] = curDistance;
  }
  
  //Sorting
  for(int x = 0; x < 8; x++) {
   for(int y = 0; y < 8-(x+1); y++) {
     if(recordDistances[y] > recordDistances[y+1]) {
       holder = recordDistances[y+1];
       recordDistances[y+1] = recordDistances[y];
       recordDistances[y] = holder;
     }
   }
  }

  return (float)recordDistances[4];
}

//float IRSensor::movingAverFilter(float curDistance){
//  float sum = 0;
//  if (numValues < 10) {
//    recordDistances[numValues] = curDistance;
//    numValues++;
//  }else {
//    for (int i = 1; i < 10; i++) {
//      recordDistances[i-1] = recordDistances[i];
//    }
//
//    recordDistances[9] = curDistance;
//  }
//
//  for (int i = 0; i < numValues; i++) {
//    sum += recordDistances[i];
//  }
//
//  return (float)sum/numValues;
//}

void IRSensor::setup(float a1, float a2) {

	chrono_.start();
	corrParam1_ = a1;
	corrParam2_ = a2;
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

float IRSensor::getCorrectLinear(float val) {
	return corrParam1_ * val + corrParam2_;
}

float IRSensor::getCorrectExp(float val) {
	return corrParam1_ * pow(val,corrParam2_);
}

