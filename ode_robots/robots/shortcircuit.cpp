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
 *   Revision 1.5  2005-11-09 13:24:42  martius
 *   added GPL
 *
 ***************************************************************************/
#include <drawstuff/drawstuff.h>
#include <ode/ode.h>
#include <assert.h>

#include "simulation.h"

#include "shortcircuit.h"


ShortCircuit::ShortCircuit(const OdeHandle& odeHandle, int sensornumber, int motornumber)
  : AbstractRobot::AbstractRobot(odeHandle){

  sensorno = sensornumber; 
  motorno  = motornumber;  
  motors = (motor*)malloc(motorno * sizeof(motor));
  for(int i=0; i < motorno; i++){
    motors[i]=0.0;
  }
  
};

/** sets actual motorcommands
    @param motors motors scaled to [-1,1] 
    @param motornumber length of the motor array
*/
void ShortCircuit::setMotors(const motor* _motors, int motornumber){
  assert(motornumber == motorno);
  memcpy(motors, _motors, sizeof(motor) * motornumber);
};

/** returns actual sensorvalues
    @param sensors sensors scaled to [-1,1] (more or less)
    @param sensornumber length of the sensor array
    @return number of actually written sensors
*/
int ShortCircuit::getSensors(sensor* sensors, int sensornumber){
  assert(sensornumber == sensorno);  
  int mini = min(sensorno,motorno); 
  for (int i=0; i< mini; i++){
    sensors[i]=motors[i]; // %motorno
  }
  for (int i=mini; i< sensorno; i++){
    sensors[i]=0;
  }
  return sensorno;
};

/** returns position of robot 
    @return position robot position in struct Position  
*/
Position ShortCircuit::getPosition(){
  Position pos;
  pos.x=0;
  pos.y=0;
  pos.z=0;
  return pos;
};

/** returns a vector with the positions of all segments of the robot
    @param poslist vector of positions (of all robot segments) 
    @return length of the list
*/
int ShortCircuit::getSegmentsPosition(vector<Position> &poslist){
  return 0;
};  

/**
 * draws the vehicle
 */
void ShortCircuit::draw(){};







