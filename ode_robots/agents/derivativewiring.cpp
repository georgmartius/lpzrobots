/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
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
 *   $Log$
 *   Revision 1.4  2005-07-26 17:02:37  martius
 *   2.derivative scaled correctly
 *
 *   Revision 1.3  2005/07/21 15:09:13  martius
 *   blind motors
 *
 *   Revision 1.2  2005/07/21 11:30:59  fhesse
 *   started with blind motors
 *
 *   Revision 1.1  2005/07/18 14:44:55  martius
 *   wiring that supports derivatives
 *
 *                                                                         *
 ***************************************************************************/

#include "derivativewiring.h"


/// constructor
DerivativeWiring::DerivativeWiring(const DerivativeWiringConf& conf, 
				   NoiseGenerator* noise)
  : AbstractWiring::AbstractWiring(noise) {

  this->conf=conf;
  time     = buffersize;
  // make sure that at least id is on.
  if (!conf.useFirstD && !conf.useSecondD) this->conf.useId=true; 
}

DerivativeWiring::~DerivativeWiring(){
  for(int i=0 ; i<buffersize; i++){
    if(sensorbuffer[i]) free(sensorbuffer[i]);
  }
  if(id) free(id);
  if(first) free(first);
  if(second) free(second);
  if(blindMotors) free(blindMotors);
}


bool DerivativeWiring::init(int rsensornumber, int rmotornumber){  
  this->rsensornumber = rsensornumber;
  this->rmotornumber  = rmotornumber;
  blindMotorNumber = this->rmotornumber * conf.blindMotorSets;

  this->csensornumber = this->rsensornumber*( (int)conf.useId+(int)conf.useFirstD+(int)conf.useSecondD)
    + blindMotorNumber;
  this->cmotornumber  = this->rmotornumber + blindMotorNumber;
    
  for(int i=0; i<buffersize; i++){
    sensorbuffer[i]      = (sensor*) malloc(sizeof(sensor) * this->rsensornumber);
    for(int k=0; k < this->rsensornumber; k++){
      sensorbuffer[i][k]=0;
    }
  }
  blindMotors       = (motor*) malloc(sizeof(motor) * (blindMotorNumber+1));
  for(unsigned int k=0; k < blindMotorNumber; k++){
    blindMotors[k]=0;
  }

  id                = (sensor*) malloc(sizeof(sensor) * this->rsensornumber);
  first             = (sensor*) malloc(sizeof(sensor) * this->rsensornumber);
  second            = (sensor*) malloc(sizeof(sensor) * this->rsensornumber);

  if(!noiseGenerator) return false;
  noiseGenerator->init(this->rsensornumber);
  return true;
}

/// Realizes a wiring from robot sensors to controller sensors. 
//   @param rsensors pointer to array of sensorvalues from robot 
//   @param rsensornumber number of sensors from robot
//   @param csensors pointer to array of sensorvalues for controller (includes derivatives if specified)
//   @param csensornumber number of sensors to controller
//   @param noise size of the noise added to the sensors
bool DerivativeWiring::wireSensors(const sensor* rsensors, int rsensornumber, 
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
  // calc smoothed sensor values
  for(int i=0; i < this->rsensornumber; i++ ){ 
      sensorbuffer[index][i] = (1-conf.eps)*sensorbuffer[lastIndex][i] + conf.eps*rsensors[i]; 
  }
 
  int offset=0;
  if(conf.useId) { // normal sensors values
    memcpy(id, rsensors, sizeof(sensor) * this->rsensornumber);
    noiseGenerator->add(id, -noise, noise);   
    //  memcpy(csensors+offset, sensorbuffer[index], sizeof(sensor) * this->rsensornumber);
    // or use noised values...
    memcpy(csensors+offset, id, sizeof(sensor) * this->rsensornumber);
    offset+=this->rsensornumber;	   
  }   
  
  if(conf.useFirstD) { // first derivative
    calcFirstDerivative();
    memcpy(csensors+offset, first, sizeof(sensor) * this->rsensornumber);
    offset+=this->rsensornumber;	   
  }
  if(conf.useSecondD) { // second derivative
    calcSecondDerivative();
    memcpy(csensors+offset, second, sizeof(sensor) * this->rsensornumber);
    // test  ( if angle near bounce point than set derivative to 0;
    // for(int i=0; i<this->rsensornumber; i++){
    //       if(fabs(*(csensors+i)) > 0.8)
    // 	*(csensors+offset+i) = 0;
    //     }
    offset+=this->rsensornumber;	   
  }      

  if(conf.blindMotorSets > 0) { // shortcircuit of blind motors
    memcpy(csensors+offset, blindMotors, sizeof(sensor) * blindMotorNumber);
    offset+=blindMotorNumber;	   
  }      
  
  if(offset!=this->csensornumber){ 
    fprintf(stderr, "%s:%i: Something strange happend!\n", __FILE__, __LINE__);  
    return false;
  } 
  time++;
  return true;
}


/// Realizes one to one wiring from controller motor outputs to robot motors. 
//   @param rmotors pointer to array of motorvalues for robot 
//   @param rmotornumber number of robot motors 
//   @param cmotors pointer to array of motorvalues from controller  
//   @param cmotornumber number of motorvalues from controller
bool DerivativeWiring::wireMotors(motor* rmotors, int rmotornumber,
				  const motor* cmotors, int cmotornumber){

  if (this->cmotornumber!=cmotornumber && rmotornumber <= cmotornumber) 
    return false;
  else{
    memcpy(rmotors, cmotors, sizeof(motor)*rmotornumber);
    if(blindMotorNumber!=0){
      memcpy(blindMotors, cmotors + rmotornumber, sizeof(motor)*blindMotorNumber);      
    }
    return true;
  }
}

/// f'(x) = (f(x+1) - f(x-1)) / 2
//  since we do not have f(x+1) we go one timestep in the past
void DerivativeWiring::calcFirstDerivative(){  
  sensor* t   = sensorbuffer[time%buffersize];
  sensor* tm2 = sensorbuffer[(time-2)%buffersize];
  for(int i=0; i < rsensornumber; i++){
    first[i] = conf.derivativeScale*(t[i] - tm2[i]);
  }
}

/// f'(x) = f(x) - 2f(x-1) + f(x-2)
void DerivativeWiring::calcSecondDerivative(){
  sensor* t   = sensorbuffer[time%buffersize];
  sensor* tm1 = sensorbuffer[(time-1)%buffersize];
  sensor* tm2 = sensorbuffer[(time-2)%buffersize];
  for(int i=0; i < rsensornumber; i++){
    second[i] = (t[i] - 2*tm1[i] + tm2[i])*conf.derivativeScale*conf.derivativeScale; 
  }
}

