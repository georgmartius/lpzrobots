/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
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
 *   Revision 1.9  2011-05-30 13:56:42  martius
 *   clean up: moved old code to oldstuff
 *   configable changed: notifyOnChanges is now used
 *    getParam,setParam, getParamList is not to be overloaded anymore
 *
 *   Revision 1.8  2005/12/21 00:19:24  robot7
 *   made the pointer to the camera handling function extern
 *   its now defined in multi_controller.cpp (i think)
 *
 *   Revision 1.7  2005/12/12 13:44:45  martius
 *   barcodesensor is working
 *
 *   Revision 1.6  2005/11/22 15:48:59  robot3
 *   inserted raceground sensors
 *
 *   Revision 1.5  2005/11/15 14:23:44  robot3
 *   raceground testet
 *
 *   Revision 1.4  2005/10/17 13:17:10  martius
 *   converted to new list's
 *
 *   Revision 1.3  2005/10/17 13:05:46  robot3
 *   std lists included
 *
 *   Revision 1.2  2005/08/09 11:06:30  robot1
 *   camera module included
 *
 *   Revision 1.1  2005/08/08 11:14:54  robot1
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
extern void (*cameraHandlingFunction)();


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

    //         printf("velocity: %f\n",velocity);
    //         printf("leftRightShift: %f\n",leftRightShift);

    if (leftRightShift!=0){
      velocity +=0.5;
    }


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
  //          if (cameraHandlingDefined==1)
  //                  cameraHandlingFunction();
  t++;
  for (int i =4; i<number_sensors;i++)
    cout << "sensor " << i << " = " << sensors[i] << "\t";
  cout << "\n";

  if (leftRightShift>0.5){
    velocity -=0.5;
  }

};


Configurable::paramval SimpleController::getParam(const paramkey& key, bool traverseChildren) const{
  if(key == "velocity") return velocity;
  else if(key == "leftRightShift") return leftRightShift;
  else  return AbstractController::getParam(key) ;
}

bool SimpleController::setParam(const paramkey& key, paramval val, bool traverseChildren){
  if(key == "velocity") {
    velocity=val;
    if(velocity == 0) leftRightShift = 0;
  }
  else if(key == "leftRightShift") leftRightShift=val;
  else return AbstractController::setParam(key, val);
  return true;
}

Configurable::paramlist SimpleController::getParamList() const{
  paramlist l;
  l += pair<paramkey, paramval> (string("velocity"), velocity);
  l += pair<paramkey, paramval> (string("leftRightShift"), leftRightShift);
  return l;
}


/** Initialises the registers the given callback functions.
    @param handling() is called every step that the camera gets new position
    and view.
*/
/*  void SimpleController::setCameraHandling(void (*handling)()) {
    cameraHandlingFunction=handling;
    // enable the camerahandling
    cameraHandlingDefined=1;
    };*/
