/*
 * Mapping.cpp
 *
 *  Created on: May 25, 2017
 *      Author: seungmin
 */

#include "Mapping.h"


Mapping::Mapping(float firstSonarIn) {
  xPos = 0;
  yPos = 0;
  xRefSonar = firstSonarIn;
}

Mapping::~Mapping() {
	// TODO Auto-generated destructor stub
}

void firstTurn(float secondSonarIn) {
  yRefSonar = secondSonarIn;
}

void saveInfo(float sonarInput, bool isX) {
  if(isX == 1) {
    xPos = xRefSonar - sonarInput;
  }
  else  {
    yPos = yRefSonar - sonarInput;
  }
}

void sendInfo() {
  
}

