/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                         *
 ***************************************************************************/

#include "selectiveone2onewiring.h"
#include <assert.h>
#include <cstring>

/// constructor
SelectiveOne2OneWiring::SelectiveOne2OneWiring(NoiseGenerator* noise,
                                               select_predicate* sel_sensor,
                                               int plotMode, const std::string& name)
  : One2OneWiring(noise, plotMode, 0, name), sel_sensor(sel_sensor) {
  assert(sel_sensor);
}

SelectiveOne2OneWiring::~SelectiveOne2OneWiring(){
  if(sel_sensor) delete sel_sensor;
}


/// initializes the number of sensors and motors on robot side, calculate
//  number of sensors and motors on controller side
bool SelectiveOne2OneWiring::initIntern(){
  One2OneWiring::initIntern();
  int num=0;
  for(int i=0; i<rsensornumber; i++){
    if((*sel_sensor)(i,rsensornumber)) num++;
  }
  csensornumber = num;
  printf("robot sensors: %i, selected senors: %i\n", rsensornumber, num);
  return true;
}

/// Realizes selective one to one wiring from robot sensors to controller sensors.
//   @param rsensors pointer to array of sensorvalues from robot
//   @param rsensornumber number of sensors from robot
//   @param csensors pointer to array of sensorvalues for controller
//   @param csensornumber number of sensors to controller
//   @param noise size of the noise added to the sensors
bool SelectiveOne2OneWiring::wireSensorsIntern(const sensor* rsensors, int rsensornumber,
                                               sensor* csensors, int csensornumber,
                                               double noiseStrength){
  // noisevals are set in AbstractWiring()
  int num=0;
  for(int i=0; i< rsensornumber; i++){
    if((*sel_sensor)(i,rsensornumber)){
      csensors[num] = rsensors[i] + noisevals[i];
      num++;
    }
  }
  return true;
}

