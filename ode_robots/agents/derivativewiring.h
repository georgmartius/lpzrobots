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
 *   Revision 1.1  2005-07-18 14:44:55  martius
 *   wiring that supports derivatives
 *
 *                                                                         *
 ***************************************************************************/
#ifndef __DERIVATIVEWIRING_H
#define __DERIVATIVEWIRING_H

#include "abstractwiring.h"

/// Abstract wiring-object between controller and robot. 
//   Implements a wiring (between controller and robot) 
//   which includes the first and second derivative 
//   of the original robot sensor values
class DerivativeWiring : public AbstractWiring{
public:
  /// constructor
  //  @param id include zeroth derivative
  //  @param first include first derivative
  //  @param second include second derivative
  //  if all boolean parametes are false, id is set to true (equivalent to One2OneWiring)
  //  @param eps update rate for floating average (0 -> no sensor variation, 1 -> no smoothing)
  //  @param noise NoiseGenerator that is used for adding noise to sensor values  
  //  @param derivativeScale factor for the derivatives (because they are usually small)
  DerivativeWiring(bool id, bool first, bool second, double eps, 
		   NoiseGenerator* noise, double derivativeScale=5);

  virtual ~DerivativeWiring();

  /// initializes the number of sensors and motors from robot, calculate
  //  number of sensors and motors on controller side
  virtual bool init(int robotsensornumber, int robotmotornumber);

  /// Realizes one to one wiring from robot sensors to controller sensors. 
  //   @param rsensors pointer to array of sensorvalues from robot 
  //   @param rsensornumber number of sensors from robot
  //   @param csensors pointer to array of sensorvalues for controller  
  //   @param csensornumber number of sensors to controller
  //   @param noise size of the noise added to the sensors
  virtual bool wireSensors(sensor* rsensors, int rsensornumber, 
			  sensor* csensors, int csensornumber,
			  double noise);

  /// Realizes one to one wiring from controller motor outputs to robot motors. 
  //   @param rmotors pointer to array of motorvalues for robot 
  //   @param rmotornumber number of robot motors 
  //   @param cmotors pointer to array of motorvalues from controller  
  //   @param cmotornumber number of motorvalues from controller
  virtual bool wireMotors(motor* rmotors, int rmotornumber,
			 motor* cmotors, int cmotornumber);

protected:

  void calcFirstDerivative();
  void calcSecondDerivative();

  double eps;
  double derivativeScale;
  static const int buffersize=5;
  int time;

  sensor* sensorbuffer[buffersize]; // current and old smoothed sensor values of robot
  sensor* first;               // current first derivative
  sensor* second;              // current second derivative
  motor *motors;               
  bool useId, useFirst, useSecond;

};

#endif
