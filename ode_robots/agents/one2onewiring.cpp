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
 *   Revision 1.2  2005-07-18 10:15:13  martius
 *   noise is added here
 *
 *   Revision 1.1  2005/07/14 15:57:54  fhesse
 *   now agent contains controller, robot and wiring, plotting ability included, therefore plotagent can be removed; ono2onewiring replaces one2oneagent
 *                                            *
 *                                                                         *
 ***************************************************************************/

#include "one2onewiring.h"


/// constructor
One2OneWiring::One2OneWiring(NoiseGenerator* noise)
  :AbstractWiring::AbstractWiring(noise){
}


/// initializes the number of sensors and motors from robot, calculate
//  number of sensors and motors on controller side
bool One2OneWiring::init(int robotsensornumber, int robotmotornumber){
  rsensornumber = robotsensornumber;
  rmotornumber  = robotmotornumber;
  csensornumber = rsensornumber;
  cmotornumber  = rmotornumber;
  if(!noiseGenerator) return false;
  noiseGenerator->init(rsensornumber);
  return true;
}

/// Realizes one to one wiring from robot sensors to controller sensors. 
//   @param rsensors pointer to array of sensorvalues from robot 
//   @param rsensornumber number of sensors from robot
//   @param csensors pointer to array of sensorvalues for controller  
//   @param csensornumber number of sensors to controller
//   @param noise size of the noise added to the sensors
int One2OneWiring::wireSensors(sensor* rsensors, int rsensornumber, 
			       sensor* csensors, int csensornumber, 
			       double noise){
  if (rsensornumber!=csensornumber)
    return false;
  else{
    noiseGenerator->add(rsensors, -noise, noise);
    memcpy(csensors, rsensors, sizeof(sensor)*rsensornumber);
    return true;
  }
}


/// Realizes one to one wiring from controller motor outputs to robot motors. 
//   @param rmotors pointer to array of motorvalues for robot 
//   @param rmotornumber number of robot motors 
//   @param cmotors pointer to array of motorvalues from controller  
//   @param cmotornumber number of motorvalues from controller
int One2OneWiring::wireMotors(motor* rmotors, int rmotornumber,
			      motor* cmotors, int cmotornumber){
  if (rmotornumber!=cmotornumber) 
    return false;
  else{
    memcpy(rmotors, cmotors, sizeof(motor)*rmotornumber);
    return true;
  }
}


