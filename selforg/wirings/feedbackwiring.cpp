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
 *   Revision 1.4  2008-04-17 14:54:45  martius
 *   randomGen added, which is a random generator with long period and an
 *    internal state. Each Agent has an instance and passed it to the controller
 *    and the wiring. This is good for
 *   a) repeatability on agent basis,
 *   b) parallel execution as done in ode_robots
 *
 *   Revision 1.3  2007/12/11 14:45:44  martius
 *   *** empty log message ***
 *
 *   Revision 1.2  2007/12/07 10:56:33  der
 *   changed method signature of generate() and add() of NoiseGenerator
 *
 *   Revision 1.1  2007/11/28 10:30:46  martius
 *   wiring with feedback connections
 *
 *                                            *
 *                                                                         *
 ***************************************************************************/

#include "feedbackwiring.h"
#include "controller_misc.h"
#include <assert.h>

/// constructor
FeedbackWiring::FeedbackWiring(NoiseGenerator* noise, Mode mode, 
			       double feedbackratio)
  : AbstractWiring(noise), mode(mode), defaultfeedbackratio(feedbackratio) {
  noisevals=0;
  motors=0;
  initialised=false;
  vmotornumber=0;
}

FeedbackWiring::~FeedbackWiring(){
  if(noisevals) free(noisevals);
  if(motors)    free(motors);
}


// initializes the number of sensors and motors from robot, calculate
//  number of sensors and motors on controller side
bool FeedbackWiring::init(int robotsensornumber, int robotmotornumber, RandGen* randGen){
  rsensornumber = robotsensornumber;
  rmotornumber  = robotmotornumber;  
  csensornumber = rsensornumber;  
  if((mode & Context) == 0) // without context mapping no additional motors 
    cmotornumber  = rmotornumber;
  else{ // with context mapping we have as many motors as sensors
    assert(rmotornumber < rsensornumber);
    vmotornumber = rsensornumber-rmotornumber;
    cmotornumber = rsensornumber;
  }

  noisevals = (sensor*) malloc(sizeof(sensor)  * this->rsensornumber);
  motors    = (motor*)  malloc(sizeof(motor)  * this->cmotornumber);
  memset(motors,0,sizeof(motor)  * this->cmotornumber);

  int feedbacknumber = ((mode & Motor) != 0)*rmotornumber + vmotornumber;
  if(feedbackratio.isNulltimesNull()){
    feedbackratio.set( feedbacknumber, 1);
    double c = defaultfeedbackratio;
    feedbackratio.toMapP(&c, constant);
  }else{
    assert(feedbackratio.getM()==feedbacknumber && feedbackratio.getN()==1);
  }

  if(!noiseGenerator) return false;
  noiseGenerator->init(rsensornumber, randGen);
  initialised=true;
  return true;
}

bool FeedbackWiring::wireSensors(const sensor* rsensors, int rsensornumber, 
				sensor* csensors, int csensornumber, 
				double noiseStrength){
  assert(rsensornumber==csensornumber);
  memset(noisevals, 0 , sizeof(sensor) * this->rsensornumber);
  noiseGenerator->add(noisevals, noiseStrength);
  
  int fi=0;
  if((mode & Motor) == 0){
    for(int i=0; i< rmotornumber; i++){
      csensors[i] = rsensors[i] + noisevals[i];
    }
  }else{
    for(int i=0; i< rmotornumber; i++, fi++){
      csensors[i] = feedbackratio.val(fi,0)*motors[i] + 
	(1-feedbackratio.val(fi,0))*(rsensors[i] + noisevals[i]);
    }
  }
  if((mode & Context) == 0){
    for(int i=rmotornumber; i< rsensornumber; i++){
      csensors[i] = rsensors[i] + noisevals[i];
    }
  }else{
    for(int i=rmotornumber; i< rmotornumber+vmotornumber; i++, fi++){
      csensors[i] = feedbackratio.val(fi,0)*motors[i] + 
	(1-feedbackratio.val(fi,0))*(rsensors[i] + noisevals[i]);
    }
  }
  return true;
}

bool FeedbackWiring::wireMotors(motor* rmotors, int rmotornumber,
			       const motor* cmotors, int cmotornumber){
  if (rmotornumber != cmotornumber-vmotornumber) 
    return false;
  else{
    memcpy(rmotors, cmotors, sizeof(motor)*rmotornumber);
    memcpy(motors,  cmotors, sizeof(motor)*cmotornumber);    
    return true;
  }
}


matrix::Matrix FeedbackWiring::getFeedbackRatio() const{
  assert(initialised);  
  return feedbackratio;
}

void FeedbackWiring::setFeedbackRatio(const matrix::Matrix& ratios){
  assert(initialised);
  assert(ratios.getM()*ratios.getN() == feedbackratio.getM()*feedbackratio.getN());
  feedbackratio.set(ratios.unsafeGetData());
}


Inspectable::iparamkeylist FeedbackWiring::getInternalParamNames() const {
  iparamkeylist l;
  char buffer[32];
  for(int i = 0; i < cmotornumber - rsensornumber; i++){
    sprintf(buffer,"yv[%d]", i);
    l += std::string(buffer);
  } 
  return l;
}

Inspectable::iparamvallist FeedbackWiring::getInternalParams() const {
  iparamvallist l;
  for(int i=0; i < cmotornumber - rsensornumber; i++){
    l += motors[rsensornumber+i];
  }
  return l;
}


