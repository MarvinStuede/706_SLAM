#include "Arduino.h"
float rad2deg(float radVal){
	return radVal * 90/M_PI;
}
float deg2rad(float degVal){
	return degVal * M_PI/90;
}
