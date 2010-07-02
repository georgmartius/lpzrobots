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
 *   Revision 1.13  2010-07-02 15:57:25  martius
 *   wirings have new initIntern signature -> less errors can be made
 *   abstractwiring generates the noise of given length
 *
 *   Revision 1.12  2009/08/05 22:32:21  martius
 *   big change:
 *       abstractwiring is responsable for providing sensors and motors
 *        and noise to the inspectable interface.
 *       external interface: unchanged except plotMode in constructor
 *       internal interface: all subclasses have to overload
 *         initIntern, wireSensorsIntern, wireMotorsIntern
 *       All existing implementation are changed
 *
 *   Revision 1.11  2009/03/27 06:16:57  guettler
 *   support for gcc 4.3 compatibility (has to be checked), StatisticTools moves from utils to statistictools
 *
 *   Revision 1.10  2008/04/17 14:54:45  martius
 *   randomGen added, which is a random generator with long period and an
 *    internal state. Each Agent has an instance and passed it to the controller
 *    and the wiring. This is good for
 *   a) repeatability on agent basis,
 *   b) parallel execution as done in ode_robots
 *
 *   Revision 1.9  2007/12/07 10:56:33  der
 *   changed method signature of generate() and add() of NoiseGenerator
 *
 *   Revision 1.8  2007/11/29 19:18:09  martius
 *   blind channels
 *
 *   Revision 1.7  2007/08/29 11:32:55  martius
 *   cleanup
 *
 *   Revision 1.6  2006/12/12 09:44:02  martius
 *   corrected wrong commit comment
 *
 *   Revision 1.5  2006/12/11 18:19:02  martius
 *   noisegen is deleted in abstractwiring
 *
 *   Revision 1.4  2006/12/11 18:17:22  martius
 *   delete instead of free for noisegen
 *
 *   Revision 1.3  2006/07/20 17:14:36  martius
 *   removed std namespace from matrix.h
 *   storable interface
 *   abstract model and invertablemodel as superclasses for networks
 *
 *   Revision 1.2  2006/07/14 12:24:02  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.2  2006/05/30 14:13:03  fhesse
 *   in getInternalParams():
 *   write noise values only when plotNoise flag is set
 *
 *   Revision 1.1.2.1  2005/11/16 11:24:28  martius
 *   moved to selforg
 *
 *   Revision 1.9  2005/10/28 12:05:54  martius
 *   new inspectable interface
 *
 *   Revision 1.8  2005/10/24 13:32:07  fhesse
 *   comments adjusted and in doxygen style
 *
 *   Revision 1.7  2005/10/06 17:11:37  martius
 *   switched to stl lists
 *
 *   Revision 1.6  2005/08/31 11:10:36  martius
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
#include <assert.h>
#include <cstring>

/// constructor
One2OneWiring::One2OneWiring(NoiseGenerator* noise, int plotMode, int blind)
  : AbstractWiring(noise,plotMode), blind(blind){
  blindmotors=0;
}

One2OneWiring::~One2OneWiring(){
  if(blindmotors) free(blindmotors);
}


/// initializes the number of sensors and motors from robot, calculate
//  number of sensors and motors on controller side
bool One2OneWiring::initIntern(){
  csensornumber = rsensornumber+blind;
  cmotornumber  = rmotornumber+blind;
  noisenumber   = csensornumber;

  if(blind){
    blindmotors = (sensor*) malloc(sizeof(sensor)  * blind);
    memset(blindmotors, 0, sizeof(sensor)  * blind);
  }

  return true;
}

/// Realizes one to one wiring from robot sensors to controller sensors. 
//   @param rsensors pointer to array of sensorvalues from robot 
//   @param rsensornumber number of sensors from robot
//   @param csensors pointer to array of sensorvalues for controller  
//   @param csensornumber number of sensors to controller
//   @param noise size of the noise added to the sensors
bool One2OneWiring::wireSensorsIntern(const sensor* rsensors, int rsensornumber, 
				      sensor* csensors, int csensornumber, 
				      double noiseStrength){
  assert(rsensornumber == this->rsensornumber);
  assert(csensornumber == this->csensornumber);
  // the noisevals are set in abstractwiring
  for(int i=0; i< rsensornumber; i++){
    csensors[i] = rsensors[i] + noisevals[i];
  }
  for(int i=0; i< blind; i++){
    csensors[i + rsensornumber] = blindmotors[i] + noisevals[rsensornumber+i];
  }
  return true;  
}


/// Realizes one to one wiring from controller motor outputs to robot motors. 
//   @param rmotors pointer to array of motorvalues for robot 
//   @param rmotornumber number of robot motors 
//   @param cmotors pointer to array of motorvalues from controller  
//   @param cmotornumber number of motorvalues from controller
bool One2OneWiring::wireMotorsIntern(motor* rmotors, int rmotornumber,
				     const motor* cmotors, int cmotornumber){
  assert(rmotornumber == this->rmotornumber);
  assert(cmotornumber == this->cmotornumber);
  memcpy(rmotors, cmotors, sizeof(motor)*rmotornumber);
  if(blind)
    memcpy(blindmotors, cmotors+rmotornumber, sizeof(motor)*blind);
  return true;
}


