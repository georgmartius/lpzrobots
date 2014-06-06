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
#include <assert.h>

#include "simulation.h"

#include "shortcircuit.h"
using namespace std;

namespace lpzrobots {

  ShortCircuit::ShortCircuit(const OdeHandle& odeHandle,
                             const OsgHandle& osgHandle, int sensornumber, int motornumber)
    : OdeRobot(odeHandle, osgHandle, "ShortCircuit", "$Id$"){

    sensorno = sensornumber;
    motorno  = motornumber;
    motors = (double*)malloc(motorno * sizeof(motor));
    for(int i=0; i < motorno; i++){
      motors[i]=0.0;
    }
    dummy = new DummyPrimitive();
    dummy->init(odeHandle,0, osgHandle);
  };

  ShortCircuit::~ShortCircuit(){
    if(motors) free(motors);
  }

  /** sets actual motorcommands
      @param _motors motors scaled to [-1,1]
      @param motornumber length of the motor array
  */
  void ShortCircuit::setMotorsIntern(const double* _motors, int motornumber){
    assert(motornumber == motorno);
    memcpy(motors, _motors, sizeof(motor) * motornumber);
  };

  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1] (more or less)
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  int ShortCircuit::getSensorsIntern(sensor* sensors, int sensornumber){
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


}
