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
 *   Revision 1.1  2005-07-06 16:06:28  martius
 *   first attempt to provide pseudosensors. here first and second derivative
 *
 ***************************************************************************/
#include "derivativeagent.h"
#include "noisegenerator.h"

DerivativeAgent::DerivativeAgent(bool useId, bool useFirstD, bool useSecondD, double _eps,
				 NoiseGenerator* noise,
				 PlotMode plotmode /* = GuiLogger*/, double _derivativeScale/*=5*/) 
  : PlotAgent(plotmode), useId(useId), useFirst(useFirstD), useSecond(useSecondD)
{
  noiseGenerator = noise;
  robotsensors = 0;
  controllersensors = 0; 
  derivativeScale = _derivativeScale;
  motors   = 0; 
  eps      = _eps;
  time     = buffersize;
  if (!useFirstD && !useSecond) useId=true; // make sure that at least id is on.
}


DerivativeAgent::~DerivativeAgent(){
  for(int i=0 ; i<buffersize; i++){
    if(sensorbuffer[i]) free(sensorbuffer[i]);
  }
  if(robotsensors) free(robotsensors);
  if(first) free(first);
  if(second) free(second);
  if(controllersensors) free(controllersensors);
  if(motors) free(motors);
  if(noiseGenerator) delete noiseGenerator;
}


bool DerivativeAgent::init(AbstractController* controller, AbstractRobot* robot){
  robotsensornumber = robot->getSensorNumber();
  motornumber  = robot->getMotorNumber();

  controllersensornumber = robotsensornumber*((int)useId+(int)useFirst+(int)useSecond);
     
  controller->init(controllersensornumber, motornumber);
  
  if(!PlotAgent::init(controller, robot)) return false;    
  
  for(int i=0; i<buffersize; i++){
    sensorbuffer[i]      = (sensor*) malloc(sizeof(sensor) * robotsensornumber);
    memset(sensorbuffer[i],0, sizeof(sensor) * robotsensornumber);
  }
  robotsensors      = (sensor*) malloc(sizeof(sensor) * robotsensornumber);
  first             = (sensor*) malloc(sizeof(sensor) * robotsensornumber);
  second            = (sensor*) malloc(sizeof(sensor) * robotsensornumber);
  controllersensors = (sensor*) malloc(sizeof(sensor) * controllersensornumber);
  motors            = (motor*)  malloc(sizeof(motor) * motornumber);
  
  if(!noiseGenerator) return false;
  noiseGenerator->init(robotsensornumber);
  //  noiseGenerator->init(controllersensornumber);
  return true;
}
 
void DerivativeAgent::step(double noise){
  if(!controller || !robot || !motors) {
    fprintf(stderr, "%s:%i: something is null: cont %x rob %x mots %x!\n", 
	    __FILE__, __LINE__, (unsigned int)controller, (unsigned int)robot, 
	    (unsigned int)motors);
  }
  int len =  robot->getSensors(robotsensors, robotsensornumber);
  if(len != robotsensornumber){
    fprintf(stderr, "%s:%i: Got not enough sensors!\n", __FILE__, __LINE__);
  }
  int index = (time) % buffersize;  
  // calc smoothed sensor values
  for(int i=0; i < robotsensornumber; i++ ){ 
      sensorbuffer[index][i] = (1-eps)*sensorbuffer[index][i] + eps*robotsensors[i]; 
  }
 
  noiseGenerator->add(robotsensors, -noise, noise);   
 
  int offset=0;
  if(useId) {
    //    memcpy(controllersensors+offset, sensorbuffer[index], sizeof(sensor) * robotsensornumber);
    memcpy(controllersensors+offset, robotsensors, sizeof(sensor) * robotsensornumber);
    offset+=robotsensornumber;	   
  }
  
  if(useFirst) {
    calcFirstDerivative();
    memcpy(controllersensors+offset, first, sizeof(sensor) * robotsensornumber);
    offset+=robotsensornumber;	   
  }
  if(useSecond) {
    calcSecondDerivative();
    memcpy(controllersensors+offset, second, sizeof(sensor) * robotsensornumber);
    offset+=robotsensornumber;	   
  }      
  if(offset!=controllersensornumber){
    fprintf(stderr, "%s:%i: Something strange happend!\n", __FILE__, __LINE__);  
    return;
  }

  // add noise
  //  noiseGenerator->add(controllersensors, -noise, noise); 
  controller->step(controllersensors, controllersensornumber, motors, motornumber);
  robot->setMotors(motors, motornumber);
  plot(controllersensors, controllersensornumber, motors, motornumber);
  time++;
}
 

/// f'(x) = (f(x+1) - f(x-1)) / 2
//  since we do not have f(x+1) we go one timestep in the past
void DerivativeAgent::calcFirstDerivative(){  
  sensor* t   = sensorbuffer[time%buffersize];
  sensor* tm2 = sensorbuffer[(time-2)%buffersize];
  for(int i=0; i < robotsensornumber; i++){
    first[i] = derivativeScale*(t[i] - tm2[i]);
  }
}

/// f'(x) = f(x) - 2f(x-1) + f(x-2)
void DerivativeAgent::calcSecondDerivative(){
  sensor* t   = sensorbuffer[time%buffersize];
  sensor* tm1 = sensorbuffer[(time-1)%buffersize];
  sensor* tm2 = sensorbuffer[(time-2)%buffersize];
  for(int i=0; i < robotsensornumber; i++){
    second[i] = (t[i] - 2*tm1[i] + tm2[i])*derivativeScale;
  }
}

