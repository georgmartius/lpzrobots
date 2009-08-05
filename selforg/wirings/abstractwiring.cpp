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
 *   Revision 1.1  2009-08-05 22:32:21  martius
 *   big change:
 *       abstractwiring is responsable for providing sensors and motors
 *        and noise to the inspectable interface.
 *       external interface: unchanged except plotMode in constructor
 *       internal interface: all subclasses have to overload
 *         initIntern, wireSensorsIntern, wireMotorsIntern
 *       All existing implementation are changed
 *
 *                                            *
 *                                                                         *
 ***************************************************************************/

#include "abstractwiring.h"



bool AbstractWiring::init(int robotsensornumber, int robotmotornumber, RandGen* randGen){    
  rsensornumber = robotsensornumber;
  rmotornumber  = robotmotornumber;
  noisevals = (sensor*) malloc(sizeof(sensor)  * this->rsensornumber);
  memset(noisevals, 0 , sizeof(sensor) * this->rsensornumber);
  if(noiseGenerator)
    noiseGenerator->init(rsensornumber, randGen);
  bool rv= initIntern(robotsensornumber, robotmotornumber, randGen);    
  mRsensors.set(rsensornumber,1);
  mRmotors.set(rmotornumber,1);
  mCsensors.set(csensornumber,1);
  mCmotors.set(cmotornumber,1);
  if(plotMode & Controller) {
    addInspectableMatrix("x", &mCsensors);
    addInspectableMatrix("y", &mCmotors);
  }
  if(plotMode & Robot) {
    addInspectableMatrix("x_R",&mRsensors);
    addInspectableMatrix("y_R",&mRmotors);
  }
  initialised = true;
  return rv;
}

bool AbstractWiring::wireSensors(const sensor* rsensors, int rsensornumber,
				 sensor* csensors, int csensornumber,
				 double noiseStrength){
  assert(initialised);
  if(noiseGenerator) {
    memset(noisevals, 0 , sizeof(sensor) * this->rsensornumber);    
    noiseGenerator->add(noisevals, noiseStrength);  
  } 
  bool rv = wireSensorsIntern(rsensors, rsensornumber, csensors, csensornumber, noiseStrength);
  mRsensors.set(rsensors);
  mCsensors.set(csensors);
  return rv;
}

bool AbstractWiring::wireMotors(motor* rmotors, int rmotornumber,
				const motor* cmotors, int cmotornumber){
  assert(initialised);
  bool rv = wireMotorsIntern(rmotors, rmotornumber, cmotors, cmotornumber);
  mRmotors.set(rmotors);
  mCmotors.set(cmotors);
  return rv;
}

Inspectable::iparamkeylist AbstractWiring::getInternalParamNames() const {
  iparamkeylist l=Inspectable::getInternalParamNames();
  char buffer[32];
  if(plotMode & Noise) {    
    for(int i = 0; i < rsensornumber; i++){
      sprintf(buffer,"n[%d]", i);
      l += std::string(buffer);
    }
  } 
  return l;
}

Inspectable::iparamvallist AbstractWiring::getInternalParams() const {
  iparamvallist l=Inspectable::getInternalParams();
  if(plotMode & Noise) {    
    for(int i=0; i < rsensornumber; i++){
      l += noisevals[i];
    }
  }
  return l;
}
