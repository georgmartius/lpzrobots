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
 *   Revision 1.1  2005-08-08 11:14:54  robot1
 *   simple control for moving robot with keyboard
 *
 *                                                                         * 
 *                                                                         *
 ***************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "simplecontroller.h"

  // pointer to the camera handling function of the user
  void (*cameraHandlingFunction)();


SimpleController::SimpleController(){
  t=0;
  velocity=0;
  leftRightShift=0;
  decreaseFactorVelocity=0.04;
  decreaseFactorShift=0.04;

  number_sensors=0;
  number_motors=0;

  // disable the camerahandling
  cameraHandlingDefined=0;
  
  // prepare name;
  Configurable::insertCVSInfo(name, "$RCSfile$", 
			            "$Revision$");
};


/** initialisation of the controller with the given sensor/ motornumber 
    Must be called before use.
*/
void SimpleController::init(int sensornumber, int motornumber){
  number_sensors=sensornumber;
  number_motors=motornumber;
};
  
/** performs one step (includes learning). 
    Calculates motor commands from sensor inputs.
    @param sensor sensors inputs scaled to [-1,1]
    @param sensornumber length of the sensor array
    @param motor motors outputs. MUST have enough space for motor values!
    @param motornumber length of the provided motor array
*/
void SimpleController::step(const sensor* sensors, int sensornumber, 
			  motor* motors, int motornumber) {
  stepNoLearning(sensors, sensornumber, motors, motornumber);
};

/** performs one step without learning. 
    @see step
*/
void SimpleController::stepNoLearning(const sensor* sensors, int number_sensors, 
				    motor* motors, int number_motors) {

  for (int i=0; i<number_motors; i++){
//     if (i%2==0)    
//       motors[i]=sin(t/velocity);
//     else
//       motors[i]=cos(t/velocity);
//    motors[i]=sin(t/velocity + i*leftRightShift*M_PI/2);

// 	printf("velocity: %f\n",velocity);
// 	printf("leftRightShift: %f\n",leftRightShift);
    
    // due to no shift all motors get equal velocity
    if (leftRightShift==0)
	motors[i]=velocity;
    // due to shift exists left and right motors get different values
    else if (leftRightShift<0) { // steering left
	if (i%2==0) // left motors
    	    motors[i]=velocity+leftRightShift*4*velocity;
	else // right motors
	    motors[i]=velocity;
    }
    else { // steering right
	if (i%2==0) // left motors
    	    motors[i]=velocity;
	else // right motors
	    motors[i]=velocity-leftRightShift*4*velocity;
    }
   }
	if (leftRightShift<0.25) {
		// if there is a big shift, velocity must be constant, otherwise
		// it will decrease
		if (velocity>=0.05) // forward
			velocity-=0.05;
		else if (velocity<=-0.05) // backward
			velocity+=0.05;
		else // has to stand still
			velocity=0;
	}
	// Werte wieder langsam zurueck setzen
	if (leftRightShift>=0.05) // left
		leftRightShift-=0.05;
	else if (leftRightShift<=-0.05) // right
		leftRightShift+=0.05;
	else // straight forward
	{
	leftRightShift=0; // only straight forward
	}
	// now call cameraHandling if defined
//  	if (cameraHandlingDefined==1)
//  		cameraHandlingFunction();
	t++;
};
  
  

paramval SimpleController::getParam(paramkey key) const{
  if(!key) return 0.0;
  if(strcmp(key, "velocity")==0) return velocity; 
  else if(strcmp(key, "leftRightShift")==0) return leftRightShift; 
  else  return AbstractController::getParam(key) ;
};

bool SimpleController::setParam(paramkey key, paramval val){
  if(!key) {
	  fprintf(stderr, "%s: empty Key!\n", __FILE__);
    return false;
  }
  if(strcmp(key, "velocity")==0) velocity=val;
  else if(strcmp(key, "leftRightShift")==0) leftRightShift=val; 
  else return AbstractController::setParam(key, val);
  return true;
};

int SimpleController::getParamList(paramkey*& keylist,paramval*& vallist) const{
  int number_params=2;  
  keylist=(paramkey*)malloc(sizeof(paramkey)*number_params);
  vallist=(paramval*)malloc(sizeof(paramval)*number_params);
  keylist[0]="velocity";
  keylist[1]="leftRightShift";
  
  vallist[0]=velocity;
  vallist[1]=leftRightShift;
  return number_params;
};

  /** Initialises the registers the given callback functions.
      @param handling() is called every step that the camera gets new position
      and view.
   */
 /*  void SimpleController::setCameraHandling(void (*handling)()) {
	   cameraHandlingFunction=handling;
  	   // enable the camerahandling
	   cameraHandlingDefined=1;
};*/




