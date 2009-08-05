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
 *   Revision 1.8  2009-08-05 22:32:21  martius
 *   big change:
 *       abstractwiring is responsable for providing sensors and motors
 *        and noise to the inspectable interface.
 *       external interface: unchanged except plotMode in constructor
 *       internal interface: all subclasses have to overload
 *         initIntern, wireSensorsIntern, wireMotorsIntern
 *       All existing implementation are changed
 *
 *   Revision 1.7  2009/03/31 15:55:35  martius
 *   check for noisegenerator to exist (excepts no noise generator as well)
 *
 *   Revision 1.6  2009/03/27 06:16:56  guettler
 *   support for gcc 4.3 compatibility (has to be checked), StatisticTools moves from utils to statistictools
 *
 *   Revision 1.5  2008/04/17 14:54:45  martius
 *   randomGen added, which is a random generator with long period and an
 *    internal state. Each Agent has an instance and passed it to the controller
 *    and the wiring. This is good for
 *   a) repeatability on agent basis,
 *   b) parallel execution as done in ode_robots
 *
 *   Revision 1.4  2007/12/07 10:56:33  der
 *   changed method signature of generate() and add() of NoiseGenerator
 *
 *   Revision 1.3  2006/12/21 11:44:17  martius
 *   commenting style for doxygen //< -> ///<
 *   FOREACH and FOREACHC are macros for collection iteration
 *
 *   Revision 1.2  2006/07/14 12:24:02  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.1  2005/11/16 11:24:28  martius
 *   moved to selforg
 *
 *   Revision 1.3  2005/10/24 13:32:07  fhesse
 *   comments adjusted and in doxygen style
 *
 *   Revision 1.2  2005/08/31 11:11:37  martius
 *   noise->noisevals
 *
 *   Revision 1.1  2005/08/22 17:28:12  martius
 *   a 1 to 1 wiring that supports the selection of some sensors only
 *
 *                                                                         *
 ***************************************************************************/

#include "selectiveone2onewiring.h"
#include "assert.h"
#include <cstring>

/// constructor
SelectiveOne2OneWiring::SelectiveOne2OneWiring(NoiseGenerator* noise, 
					       select_predicate* sel_sensor)
  : One2OneWiring(noise), sel_sensor(sel_sensor) {
}

SelectiveOne2OneWiring::~SelectiveOne2OneWiring(){
  if(sel_sensor) delete sel_sensor;
}


/// initializes the number of sensors and motors on robot side, calculate
//  number of sensors and motors on controller side
bool SelectiveOne2OneWiring::initIntern(int robotsensornumber, int robotmotornumber, RandGen* randGen){
  One2OneWiring::init(robotsensornumber, robotmotornumber, randGen);
  int num=0;
  for(int i=0; i<robotsensornumber; i++){
    if((*sel_sensor)(i,robotsensornumber)) num++;
  }  
  csensornumber = num;
  printf("robot sensors: %i, selected senors: %i\n", robotsensornumber, num);
  return true;
}

/// Realizes selective one to one wiring from robot sensors to controller sensors. 
//   @param rsensors pointer to array of sensorvalues from robot 
//   @param rsensornumber number of sensors from robot
//   @param csensors pointer to array of sensorvalues for controller  
//   @param csensornumber number of sensors to controller
//   @param noise size of the noise added to the sensors
bool SelectiveOne2OneWiring::wireSensorsIntern(const sensor* rsensors, int rsensornumber, 
					       sensor* csensors, int csensornumber, 
					       double noiseStrength){
  // noisevals are set in AbstractWiring()
  int num=0;
  for(int i=0; i< rsensornumber; i++){
    if((*sel_sensor)(i,rsensornumber)){
      csensors[num] = rsensors[i] + noisevals[i];
      num++;
    }
  }
  return true;
}

