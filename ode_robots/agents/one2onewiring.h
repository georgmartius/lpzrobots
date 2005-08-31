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
 *   Revision 1.6  2005-08-31 11:10:52  martius
 *   noise -> noisevals
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
 *   Revision 1.2  2005/07/18 10:15:03  martius
 *   noise is added here
 *
 *   Revision 1.1  2005/07/14 15:57:54  fhesse
 *   now agent contains controller, robot and wiring, plotting ability included, therefore plotagent can be removed; ono2onewiring replaces one2oneagent
 *                                            *
 *                                                                         *
 ***************************************************************************/
#ifndef __ONE2ONEWIRING_H
#define __ONE2ONEWIRING_H

#include "abstractwiring.h"

/// Abstract wiring-object between controller and robot. 
//   Implements one to one wireing of robot sensors to inputs of the controller 
//   and controller outputs to robot motors. 
class One2OneWiring :public AbstractWiring{
public:
  /// constructor
  // @param noise NoiseGenerator that is used for adding noise to sensor values  
  One2OneWiring(NoiseGenerator* noise, bool plotNoise=false);
  virtual ~One2OneWiring();

  /// initializes the number of sensors and motors from robot, calculate
  //  number of sensors and motors on controller side
  virtual bool init(int robotsensornumber, int robotmotornumber);

  /// Realizes one to one wiring from robot sensors to controller sensors. 
  //   @param rsensors pointer to array of sensorvalues from robot 
  //   @param rsensornumber number of sensors from robot
  //   @param csensors pointer to array of sensorvalues for controller  
  //   @param csensornumber number of sensors to controller
  //   @param noise size of the noise added to the sensors
  virtual bool wireSensors(const sensor* rsensors, int rsensornumber, 
			   sensor* csensors, int csensornumber,
			   double noise);

  /// Realizes one to one wiring from controller motor outputs to robot motors. 
  //   @param rmotors pointer to array of motorvalues for robot 
  //   @param rmotornumber number of robot motors 
  //   @param cmotors pointer to array of motorvalues from controller  
  //   @param cmotornumber number of motorvalues from controller
  virtual bool wireMotors(motor* rmotors, int rmotornumber,
			  const motor* cmotors, int cmotornumber);

  /** The list of the names of all internal parameters given by getInternalParams().
      @param: keylist (do NOT free it! It is a pointer to an internal structure)
      @return: length of the lists
  */
  virtual int getInternalParamNames(paramkey*& keylist);
  /** The list of the names of all internal parameters given by getInternalParams().
      @param vallist stores the values of all internal parameters 
      (in the order given by getInternalParamNames())
      @param length length of vallist array
      @return: number of parameters actually written
   */
  virtual int getInternalParams(paramval* vallist, int length);

protected:
  bool plotNoise;
  sensor* noisevals;
  paramkey* keylist;

};

#endif
