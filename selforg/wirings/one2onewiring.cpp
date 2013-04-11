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

#include "one2onewiring.h"
#include <assert.h>
#include <cstring>

/// constructor
One2OneWiring::One2OneWiring(NoiseGenerator* noise, int plotMode, int blind, const std::string& name)
  : AbstractWiring(noise, plotMode, name), blind(blind){
  blindmotors=0;
}

One2OneWiring::~One2OneWiring(){
  if(blindmotors) free(blindmotors);
}


/// initializes the number of sensors and motors from robot, calculate
//  number of sensors and motors on controller side
bool One2OneWiring::initIntern(){
  csensornumber = rsensornumber+blind;
  cmotornumber  = rmotornumber+blind;
  noisenumber   = csensornumber;

  if(blind){
    blindmotors = (sensor*) malloc(sizeof(sensor)  * blind);
    memset(blindmotors, 0, sizeof(sensor)  * blind);
  }

  return true;
}

/// Realizes one to one wiring from robot sensors to controller sensors.
//   @param rsensors pointer to array of sensorvalues from robot
//   @param rsensornumber number of sensors from robot
//   @param csensors pointer to array of sensorvalues for controller
//   @param csensornumber number of sensors to controller
//   @param noise size of the noise added to the sensors
bool One2OneWiring::wireSensorsIntern(const sensor* rsensors, int rsensornumber,
                                      sensor* csensors, int csensornumber,
                                      double noiseStrength){
  assert(rsensornumber == this->rsensornumber);
  assert(csensornumber == this->csensornumber);
  // the noisevals are set in abstractwiring
  for(int i=0; i< rsensornumber; i++){
    csensors[i] = rsensors[i] + noisevals[i];
  }
  for(int i=0; i< blind; i++){
    csensors[i + rsensornumber] = blindmotors[i] + noisevals[rsensornumber+i];
  }
  return true;
}


/// Realizes one to one wiring from controller motor outputs to robot motors.
//   @param rmotors pointer to array of motorvalues for robot
//   @param rmotornumber number of robot motors
//   @param cmotors pointer to array of motorvalues from controller
//   @param cmotornumber number of motorvalues from controller
bool One2OneWiring::wireMotorsIntern(motor* rmotors, int rmotornumber,
                                     const motor* cmotors, int cmotornumber){
  assert(rmotornumber == this->rmotornumber);
  assert(cmotornumber == this->cmotornumber);
  memcpy(rmotors, cmotors, sizeof(motor)*rmotornumber);
  if(blind)
    memcpy(blindmotors, cmotors+rmotornumber, sizeof(motor)*blind);
  return true;
}


