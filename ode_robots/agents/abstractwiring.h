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
 *   Revision 1.4  2005-07-21 15:14:47  martius
 *   wireSensors and wireMotors get constant fields
 *
 *   Revision 1.3  2005/07/18 14:44:27  martius
 *   noise moved into wiring
 *
 *   Revision 1.2  2005/07/18 10:14:45  martius
 *   noise is added here
 *
 *   Revision 1.1  2005/07/14 15:57:53  fhesse
 *   now agent contains controller, robot and wiring, plotting ability included, therefore plotagent can be removed; ono2onewiring replaces one2oneagent
 *                                            *
 *                                                                         *
 ***************************************************************************/
#ifndef __ABSTRACTWIRING_H
#define __ABSTRACTWIRING_H

#include "abstractrobot.h"
#include "noisegenerator.h"

/// Abstract wiring-object between controller and robot. 
//   Implements wiring of robot sensors to inputs of the controller and
//   controller outputs to robot motors. 
class AbstractWiring {
public:
  /// constructor
  // @param noise NoiseGenerator that is used for adding noise to sensor values
  AbstractWiring(NoiseGenerator* noise){
    rsensornumber = 0;
    rmotornumber  = 0;
    csensornumber = 0;
    cmotornumber  = 0;
    noiseGenerator = noise;
  }

  virtual ~AbstractWiring(){
    if(noiseGenerator) delete noiseGenerator;
  }

  /// Initializes the number of sensors and motors from robot, calculate
  //  number of sensors and motors on controller side.
  //  Must be overloaded to calculate and provide the appropriate numbers
  //  controllersensornumber (csensornumber) and controllermotornumber (cmotornumber).
  //  Also rmotornumber and rsensornumber should be set.
  virtual bool init(int robotsensornumber, int robotmotornumber) = 0;

  /// Realizes wiring from robot sensors to controller sensors. 
  //   Must be overloaded in order to implement the appropriate mapping. 
  //   @param rsensors pointer to array of sensorvalues from robot 
  //   @param rsensornumber number of sensors from robot
  //   @param csensors pointer to array of sensorvalues for controller  
  //   @param csensornumber number of sensors to controller
  //   @param noise size of the noise added to the sensors
  //   @return returns false if error, else true
  virtual bool wireSensors(const sensor* rsensors, int rsensornumber, 
			   sensor* csensors, int csensornumber,
			   double noise) = 0;

  /// Realizes wiring from controller motor outputs to robot motors. 
  //   Must be overloaded in order to implement the appropriate mapping. 
  //   @param rmotors pointer to array of motorvalues for robot 
  //   @param rmotornumber number of robot motors 
  //   @param cmotors pointer to array of motorvalues from controller  
  //   @param cmotornumber number of motorvalues from controller
  //   @return returns false if error, else true
  virtual bool wireMotors(motor* rmotors, int rmotornumber,
			  const motor* cmotors, int cmotornumber)  = 0;

  /// Returns the number of sensors on robot side.
  virtual int getRobotSensornumber(){return rsensornumber;}

  /// Returns the number of motors on robot side.
  virtual int getRobotMotornumber() {return rmotornumber;}

  /// Returns the number of sensors on controller side.
  virtual int getControllerSensornumber(){return csensornumber;}

  /// Returns the number of motors on controller side.
  virtual int getControllerMotornumber() {return cmotornumber;}

protected:
  int rsensornumber;  // number of robot sensors
  int rmotornumber;   // number of robot motors
  int csensornumber;  // number of controller sensors
  int cmotornumber;   // number of controller motors

  NoiseGenerator* noiseGenerator;

};

#endif
