/*
 * Mapping.h
 *
 *  Created on: May 25, 2017
 *      Author: seungmin
 */

#include <Arduino.h>
#include <LightChrono.h>

#ifndef MAPPING_H_
#define MAPPING_H_

class Mapping {

public:
  Mapping();
  virtual ~Mapping();
  float robotPosition(float xPos, float yPos, float radAngle);
  float obstaclePosition(float xPos, float yPos, float *IRsensors, float sonar);
  void sendInfo();
private:
  float currxPos;
  float curryPos;
};

#endif /* MAPPING_H_ */

