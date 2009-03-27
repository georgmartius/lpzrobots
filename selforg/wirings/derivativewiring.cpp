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
 *   Revision 1.10  2009-03-27 06:16:56  guettler
 *   support for gcc 4.3 compatibility (has to be checked), StatisticTools moves from utils to statistictools
 *
 *   Revision 1.9  2008/04/17 14:54:45  martius
 *   randomGen added, which is a random generator with long period and an
 *    internal state. Each Agent has an instance and passed it to the controller
 *    and the wiring. This is good for
 *   a) repeatability on agent basis,
 *   b) parallel execution as done in ode_robots
 *
 *   Revision 1.8  2007/12/07 10:56:33  der
 *   changed method signature of generate() and add() of NoiseGenerator
 *
 *   Revision 1.7  2006/12/11 18:23:21  martius
 *   changed order again: first all sensors, then all derivatives ...
 *   noise is only added to first sensor set
 *   now 2 functions for default configs
 *   blind motors not as sets, but as direct number given
 *
 *   Revision 1.6  2006/12/04 17:44:18  martius
 *   still completely unclear
 *
 *   Revision 1.5  2006/12/04 16:04:43  der
 *   fix
 *
 *   Revision 1.4  2006/08/10 11:56:15  martius
 *   noise is now applied to all sensors
 *
 *   Revision 1.3  2006/07/20 17:14:36  martius
 *   removed std namespace from matrix.h
 *   storable interface
 *   abstract model and invertablemodel as superclasses for networks
 *
 *   Revision 1.2  2006/07/14 12:24:02  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.3  2006/02/08 16:20:21  martius
 *   delay increased by 1
 *
 *   Revision 1.1.2.2  2006/01/30 14:13:56  martius
 *   order has changed from id_1,..,id_n, first_1,...first_n, ...
 *    to id_1,first_1, second_1, id_2, first2, .... id_n, first_n, second_n
 *
 *   Revision 1.1.2.1  2005/11/16 11:24:27  martius
 *   moved to selforg
 *
 *   Revision 1.7  2005/11/14 12:48:08  martius
 *   optimised
 *
 *   Revision 1.6  2005/10/28 12:05:27  martius
 *   adapted time horizont for derivative
 *    to quater of the time horizont of averaging
 *
 *   Revision 1.5  2005/10/24 11:06:33  fhesse
 *   comments adjusted and in doxygen style
 *
 *   Revision 1.4  2005/07/26 17:02:37  martius
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
#include <cstring>
using namespace std;

/// constructor
DerivativeWiring::DerivativeWiring(const DerivativeWiringConf& conf, 
				   NoiseGenerator* noise)
  : AbstractWiring::AbstractWiring(noise), conf(conf){

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


bool DerivativeWiring::init(int rsensornumber, int rmotornumber, RandGen* randGen){  
  this->rsensornumber = rsensornumber;
  this->rmotornumber  = rmotornumber;

  this->csensornumber = this->rsensornumber*( (int)conf.useId+(int)conf.useFirstD+(int)conf.useSecondD)
    + conf.blindMotors;
  this->cmotornumber  = this->rmotornumber + conf.blindMotors;
    
  for(int i=0; i<buffersize; i++){
    sensorbuffer[i]      = (sensor*) malloc(sizeof(sensor) * this->rsensornumber);
    for(int k=0; k < this->rsensornumber; k++){
      sensorbuffer[i][k]=0;
    }
  }
  // if(conf.useId)     id           = (sensor*) malloc(sizeof(sensor) * this->rsensornumber);
  if(conf.useFirstD)  first  = (sensor*) malloc(sizeof(sensor) * this->rsensornumber);
  if(conf.useSecondD) second = (sensor*) malloc(sizeof(sensor) * this->rsensornumber);
  if(conf.blindMotors>0){
    blindMotors       = (motor*) malloc(sizeof(motor) * conf.blindMotors);
    for(unsigned int k=0; k < conf.blindMotors; k++){
      blindMotors[k]=0;
    }
  }

  if(!noiseGenerator) return false;
  noiseGenerator->init(this->rsensornumber,randGen);
    //noiseGenerator->init(this->rsensornumber*(conf.useId+conf.useFirstD+conf.useSecondD),randGen);
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
  noiseGenerator->add(csensors, noise);   

  if(conf.blindMotors > 0) { // shortcircuit of blind motors
    int offset = (conf.useId + conf.useFirstD + conf.useSecondD)*this->rsensornumber;    
    memcpy(csensors+offset, blindMotors, sizeof(sensor) * conf.blindMotors);
  }      
  
  time++;
  return true;
}


/// Realizes wiring from controller motor outputs to robot motors. 
//   @param rmotors pointer to array of motorvalues for robot 
//   @param rmotornumber number of robot motors 
//   @param cmotors pointer to array of motorvalues from controller  
//   @param cmotornumber number of motorvalues from controller
bool DerivativeWiring::wireMotors(motor* rmotors, int rmotornumber,
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

