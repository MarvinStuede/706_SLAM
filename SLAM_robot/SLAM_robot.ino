#include "Arduino.h"
#include "IRSensor.h"
#include "UltraSonicSensor.h"
#include "I2Cdev.h"
#include "MPU.h"
#include "Kalman.h"
#include "MobilePlatform.h"

MPU mpu;
UltraSonicSensor ultrasonic;
IRSensor IR_front_left(SHARP_DX,A1);
IRSensor IR_front_right(SHARP_DX,A2);
IRSensor IR_side_front(SHARP_Ya,A3);
IRSensor IR_side_back(SHARP_Ya,A4);
MobilePlatform robot;

Kalman kalmanZ;
uint32_t timer;
float mag[3];
float gyr[3];
double dt = 0;

//float FL[9];
//float FR[9];
//float SF[9];
//float SB[9];
float angle = 0;
float kalAngle = 0;
double startTime = 0;
double prevTime = 0;
bool firstLoop = true;
float roll = 0;
float pitch = 0;
float yaw = 0;

void setup()
{
  prevTime = ((double)micros())/1000000;
	Serial.begin(115200);

	IR_front_left.setup(1.4197,-2.8392);
	IR_front_right.setup(1.1074,-0.4708);
	IR_side_front.setup(2.0813,-16.074);
	IR_side_back.setup(0.218,1.5159);
	ultrasonic.setup();
	mpu.setup();
	robot.setup();
  kalmanZ.setAngle(0);
}


void loop()
{
//  if (firstLoop) {
  //  dt = (double)(micros()/1000000) - startTime;
  //  prevTime = startTime + dt;
  //  firstLoop = false;
//  }else {
    dt = ((double)micros())/1000000 - prevTime;
    prevTime = ((double)micros())/1000000;
  //}
	
	mpu.readRegisters();
	float calFl,calFr,calSf,calSb;
  //mpu.getMag(mag);
	mpu.getGyro(gyr);
 mpu.getRPY(roll,pitch,yaw);
 angle += gyr[2] * dt;

	//Serial.print(gyr[0]);
	//Serial.print(" ");
	//Serial.print(gyr[1]);
	//Serial.print(" ");
	Serial.println(yaw,6);
	//Serial.print(" ");
    //Serial.print(angle);
  //Serial.print(" ");
	//Serial.println();

	//		Serial.print(ultrasonic.getDistance());
	//		Serial.print(" ");
	//		Serial.println();

//	Serial.print("IR_fl: ");
//	calFl = IR_front_left.getValue(LINEAR);
//  Serial.print(calFl);
////	Serial.print(medianFilter(calFl,1));
//	Serial.print(", IR_fr: ");
//	calFr = IR_front_right.getValue(LINEAR);
//  Serial.print(calFr);
////    Serial.print( IR_front_right.movingMedianFilter(calFr) );
////	Serial.print(medianFilter(calFr,2));
//	Serial.print(", IR_sf: ");
//	calSf = IR_side_front.getValue(LINEAR);
//  Serial.print(calSf);
////    Serial.print( IR_side_front.movingMedianFilter(calSf) );
////	Serial.print(medianFilter(calSf,3));
//	Serial.print(", IR_sb: ");
//	calSb = IR_side_back.getValue(EXPONENTIAL);
//  Serial.print(calSb);
////    Serial.print( IR_side_back.movingMedianFilter(calSb) );
////	Serial.print(medianFilter(calSb,4));
//	Serial.println();


	delay(10);
}

//float medianFilter(float cal, int type)
//{
//  //Size of 9
//  float arrayofcal[9];
//  float holder;
//
//  switch(type) {
//    case 1:
//      arrayofcal = FL;
//      break;
//    case 2:
//      arrayofcal = FR;
//      break;
//    case 3:
//      arrayofcal = SF;
//      break;
//    case 4:
//      arrayofcal = SB;
//      break;
//  }
//  
//  for (i = 1; i < 9; i++) {
//    arrayofcal[i-1] = arrayofcal[i];
//  }
//  arrayofcal[8] = cal;
//
//  //Sorting
//  for(x = 0; x < 8; x++) {
//   for(y = 0; y < 8-(x+1); y++) {
//     if(arrayofcal[y] > arrayofcal[y+1]) {
//       holder = arrayofcal[y+1];
//       arrayofcal[y+1] = arrayofcal[y];
//       arrayofcal[y] = holder;
//     }
//   }
//  }
//
//  switch(type) {
//    case 1:
//      FL = arrayofcal;
//      break;
//    case 2:
//      FR = arrayofcal;
//      break;
//    case 3:
//      SF = arrayofcal;
//      break;
//    case 4:
//      SB = arrayofcal;
//      break;
//  }
//  return arrayofcal[4];
//}





















