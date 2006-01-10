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
 *   Revision 1.5.4.3  2006-01-10 20:32:51  martius
 *   moved to osg
 *
 *   Revision 1.5.4.2  2005/11/15 12:29:27  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.5.4.1  2005/11/14 17:37:18  martius
 *   moved to selforg
 *
 *   Revision 1.5  2005/11/09 13:24:42  martius
 *   added GPL
 *
 ***************************************************************************/
#include <assert.h>

#include "simulation.h"

#include "shortcircuit.h"

namespace lpzrobots {

  ShortCircuit::ShortCircuit(const OdeHandle& odeHandle, 
			     const OsgHandle& osgHandle, int sensornumber, int motornumber)
    : OdeRobot(odeHandle, osgHandle){

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


}
