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
 *   Revision 1.6  2005-08-31 11:10:36  martius
 *   removed bug that causes segfault, it was in malloc of noise(vals)
 *
 *   Revision 1.5  2005/08/03 20:34:58  martius
 *   use if Inspectable interface
 *
 *   Revision 1.4  2005/07/21 15:14:47  martius
 *   wireSensors and wireMotors get constant fields
 *
 *   Revision 1.3  2005/07/18 14:44:27  martius
 *   noise moved into wiring
 *
 *   Revision 1.2  2005/07/18 10:15:13  martius
 *   noise is added here
 *
 *   Revision 1.1  2005/07/14 15:57:54  fhesse
 *   now agent contains controller, robot and wiring, plotting ability included, therefore plotagent can be removed; ono2onewiring replaces one2oneagent
 *                                            *
 *                                                                         *
 ***************************************************************************/

#include "one2onewiring.h"


/// constructor
One2OneWiring::One2OneWiring(NoiseGenerator* noise, bool plotNoise)
  : AbstractWiring(noise), plotNoise(plotNoise){
  keylist=0;
  noisevals=0;
}

One2OneWiring::~One2OneWiring(){
  if(noisevals) delete (noisevals);
  if(keylist) {
    for(int i = 0; i < rsensornumber; i++)
      free(keylist[i]);
    free(keylist);
  }
}


/// initializes the number of sensors and motors from robot, calculate
//  number of sensors and motors on controller side
bool One2OneWiring::init(int robotsensornumber, int robotmotornumber){
  rsensornumber = robotsensornumber;
  rmotornumber  = robotmotornumber;
  csensornumber = rsensornumber;
  cmotornumber  = rmotornumber;

  noisevals     = (sensor*) malloc(sizeof(sensor)  * this->rsensornumber);

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
bool One2OneWiring::wireSensors(const sensor* rsensors, int rsensornumber, 
				sensor* csensors, int csensornumber, 
				double noiseStrength){
  if (rsensornumber!=csensornumber)
    return false;
  else{
    memset(noisevals, 0 , sizeof(sensor) * this->rsensornumber);
    noiseGenerator->add(noisevals, -noiseStrength, noiseStrength);   
    for(int i=0; i< rsensornumber; i++){
      csensors[i] = rsensors[i] + noisevals[i];
    }
    return true;
  }
}


/// Realizes one to one wiring from controller motor outputs to robot motors. 
//   @param rmotors pointer to array of motorvalues for robot 
//   @param rmotornumber number of robot motors 
//   @param cmotors pointer to array of motorvalues from controller  
//   @param cmotornumber number of motorvalues from controller
bool One2OneWiring::wireMotors(motor* rmotors, int rmotornumber,
			       const motor* cmotors, int cmotornumber){
  if (rmotornumber!=cmotornumber) 
    return false;
  else{
    memcpy(rmotors, cmotors, sizeof(motor)*rmotornumber);
    return true;
  }
}

/** The list of the names of all internal parameters given by getInternalParams().
    @param: keylist (do NOT free it! It is a pointer to an internal structure)
    @return: length of the lists
*/
int One2OneWiring::getInternalParamNames(paramkey*& _keylist){
  if(plotNoise) {
    if (keylist==0){
      keylist=(paramkey*)malloc(sizeof(paramkey)*rsensornumber);
      for(int i = 0; i < rsensornumber; i++){
	keylist[i] = (paramkey) malloc(9*sizeof(char));
	sprintf(keylist[i],"n[%d]", i);
      }
    }
    _keylist=keylist;
    return rsensornumber;
  } else return 0;
}

/** The list of the names of all internal parameters given by getInternalParams().
    @param vallist stores the values of all internal parameters 
    (in the order given by getInternalParamNames())
    @param length length of vallist array
    @return: number of parameters actually written
*/
int One2OneWiring::getInternalParams(paramval* vallist, int length){
  int mini = min(length, rsensornumber);
  for(int i=0; i < mini; i++){
    vallist[i] = noisevals[i];
  }
  return mini;
}


