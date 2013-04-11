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

#include "derivativewiring.h"
#include <cstring>
using namespace std;

/// constructor
DerivativeWiring::DerivativeWiring(const DerivativeWiringConf& conf,
                                   NoiseGenerator* noise, const std::string& name)
  : AbstractWiring::AbstractWiring(noise, Controller, name), conf(conf){

  time     = buffersize;
  first    = 0;
  second   = 0;
  blindMotors = 0;
  //  this->conf.derivativeScale*= 1/this->conf.eps+0.01;
  // delay    = min(buffersize/2-1, int(0.25/(conf.eps+0.01))+1);
  // make sure that at least id is on.
  if ((!conf.useFirstD) && (!conf.useSecondD)) this->conf.useId=true;
}

DerivativeWiring::~DerivativeWiring(){
  for(int i=0 ; i<buffersize; i++){
    if(sensorbuffer[i]) free(sensorbuffer[i]);
  }
  //  if(id) free(id);
  if(first) free(first);
  if(second) free(second);
  if(blindMotors) free(blindMotors);
}


bool DerivativeWiring::initIntern(){
  csensornumber = rsensornumber*( (int)conf.useId+(int)conf.useFirstD+(int)conf.useSecondD)
    + conf.blindMotors;
  cmotornumber  = rmotornumber + conf.blindMotors;

  for(int i=0; i<buffersize; i++){
    sensorbuffer[i]      = (sensor*) malloc(sizeof(sensor) * this->rsensornumber);
    for(int k=0; k < rsensornumber; k++){
      sensorbuffer[i][k]=0;
    }
  }
  // if(conf.useId)     id           = (sensor*) malloc(sizeof(sensor) * rsensornumber);
  if(conf.useFirstD)  first  = (sensor*) malloc(sizeof(sensor) * this->rsensornumber);
  if(conf.useSecondD) second = (sensor*) malloc(sizeof(sensor) * this->rsensornumber);
  if(conf.blindMotors>0){
    blindMotors       = (motor*) malloc(sizeof(motor) * conf.blindMotors);
    for(unsigned int k=0; k < conf.blindMotors; k++){
      blindMotors[k]=0;
    }
  }

  return true;
}

/// Realizes a wiring from robot sensors to controller sensors.
//   @param rsensors pointer to array of sensorvalues from robot
//   @param rsensornumber number of sensors from robot
//   @param csensors pointer to array of sensorvalues for controller (includes derivatives if specified)
//   @param csensornumber number of sensors to controller
//   @param noise size of the noise added to the sensors
bool DerivativeWiring::wireSensorsIntern(const sensor* rsensors, int rsensornumber,
                                         sensor* csensors, int csensornumber,
                                         double noise){
  if(rsensornumber != this->rsensornumber || this->csensornumber != csensornumber){
    fprintf(stderr, "%s:%i: Wrong sensornumbers! Robot: Expected: %i, Got: %i Controller: Expected: %i, Got: %i \n",
            __FILE__, __LINE__,
            this->rsensornumber, rsensornumber,
            this->csensornumber, csensornumber);
    return false;
  }
  int index = (time) % buffersize;
  int lastIndex = (time-1) % buffersize;

  if(conf.useFirstD || conf.useSecondD){ // calc smoothed sensor values
    for(int i=0; i < this->rsensornumber; i++ ){
      sensorbuffer[index][i] = (1-conf.eps)*sensorbuffer[lastIndex][i] + conf.eps*rsensors[i];
    }
  }

  if(conf.useId) { // normal sensors values
    memcpy(csensors, rsensors, sizeof(sensor) * this->rsensornumber);
  }

  if(conf.useFirstD) { // first derivative
    calcFirstDerivative();
    int offset = conf.useId*this->rsensornumber;
    memcpy(csensors+offset, first, sizeof(sensor) * this->rsensornumber);
  }
  if(conf.useSecondD) { // second derivative
    calcSecondDerivative();
    int offset = (conf.useId + conf.useFirstD)*this->rsensornumber;
    memcpy(csensors+offset, second, sizeof(sensor) * this->rsensornumber);
  }


//   int blocksize = conf.useId + conf.useFirstD + conf.useSecondD;
//   if(conf.useId) { // normal sensors values
//     memcpy(id, rsensors, sizeof(sensor) * this->rsensornumber);
//     for(int i=0; i < this->rsensornumber; i++ ){
//       csensors[i*blocksize] = id[i];
//     }
//   }

//   if(conf.useFirstD) { // first derivative
//     calcFirstDerivative();
//     int offset = conf.useId;
//     for(int i=0; i < this->rsensornumber; i++ ){
//       csensors[i*blocksize+offset] = first[i];
//     }
//   }
//   if(conf.useSecondD) { // second derivative
//     calcSecondDerivative();
//     int offset = conf.useId + conf.useFirstD;
//     for(int i=0; i < this->rsensornumber; i++ ){
//       csensors[i*blocksize+offset] = second[i];
//     }
//   }

  // add noise only to first used sensors
  for(int i=0; i< rsensornumber; i++){
    csensors[i] = csensors[i] + noisevals[i];
  }

  if(conf.blindMotors > 0) { // shortcircuit of blind motors
    int offset = (conf.useId + conf.useFirstD + conf.useSecondD)*this->rsensornumber;
    memcpy(csensors+offset, blindMotors, sizeof(sensor) * conf.blindMotors);
  }

  time++;
  return true;
}

void DerivativeWiring::reset(){
  for(int i=0 ; i<buffersize; i++){
    if(sensorbuffer[i]) memset(sensorbuffer[i],0,sizeof(sensor) * this->rsensornumber);
  }
}


/// Realizes wiring from controller motor outputs to robot motors.
//   @param rmotors pointer to array of motorvalues for robot
//   @param rmotornumber number of robot motors
//   @param cmotors pointer to array of motorvalues from controller
//   @param cmotornumber number of motorvalues from controller
bool DerivativeWiring::wireMotorsIntern(motor* rmotors, int rmotornumber,
                                        const motor* cmotors, int cmotornumber){

  assert( (this->cmotornumber==cmotornumber) && ((rmotornumber + (signed)conf.blindMotors) == cmotornumber)) ;
  memcpy(rmotors, cmotors, sizeof(motor)*rmotornumber);
  if(conf.blindMotors>0){
    memcpy(blindMotors, cmotors + rmotornumber, sizeof(motor)*conf.blindMotors);
  }
  return true;
}

/// f'(x) = (f(x+1) - f(x-1)) / 2
//  since we do not have f(x+1) we go one timestep in the past
void DerivativeWiring::calcFirstDerivative(){
  sensor* t   = sensorbuffer[time%buffersize];
  sensor* tmdelay = sensorbuffer[(time-1)%buffersize];
  for(int i=0; i < rsensornumber; i++){
    first[i] = conf.derivativeScale*(t[i] - tmdelay[i]);
  }
}

/// f'(x) = f(x) - 2f(x-1) + f(x-2)
void DerivativeWiring::calcSecondDerivative(){
  sensor* t   = sensorbuffer[time%buffersize];
  sensor* tmdelay = sensorbuffer[(time-1)%buffersize];
  sensor* tm2delay = sensorbuffer[(time-2)%buffersize];
  for(int i=0; i < rsensornumber; i++){
    second[i] = (t[i] - 2*tmdelay[i] + tm2delay[i])*conf.derivativeScale*conf.derivativeScale;
  }
}

