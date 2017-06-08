#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2017-06-08 22:22:21

#include "Arduino.h"
#include "Arduino.h"
#include "IRSensor.h"
#include "UltraSonicSensor.h"
#include "I2Cdev.h"
#include "MPU.h"
#include "MobilePlatform.h"
#include "PIDController.h"
#include "LightChrono.h"
#include "Chrono.h"
#include "Util.h"
#include "Mapping.h"
void setup() ;
bool turnAngle(float angleGoal) ;
bool noObjectToSide(float dt, float threshold) ;
inline float rad2deg(float radVal) ;
inline float deg2rad(float degVal) ;
void toState(statesMain state) ;
void loop() ;

#include "SLAM_robot.ino"


#endif
