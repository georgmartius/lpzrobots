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
#ifndef __FORCEBOOSTWIRING_H
#define __FORCEBOOSTWIRING_H

#include "abstractwiring.h"
#include "matrix.h"
#include "configurable.h"

/** Implements one to one wiring that integrates the mismatch
    between motor commands (understood as target values) and
    sensor values and adds them to the controller output
    If more sensors than motors are used it uses just the first m sensors.
 */
class ForceBoostWiring: public AbstractWiring, public Configurable {
public:
  /** constructor
      @param noise NoiseGenerator that is used for adding noise to sensor values
      @param plotMode see AbstractWiring
      @param boost factor for mismatch integration
      @param exportBoostError whether to export force boost error to inspectables
        (guilogger)
   */
  ForceBoostWiring(NoiseGenerator* noise, double boost=0, bool exportBoostError=false,
                   int plotMode=Controller, const std::string& name = "ForceBoostWiring");

  /** destructor
   */
  virtual ~ForceBoostWiring();

  virtual void reset();
protected:

  /** initializes the number of sensors and motors on robot side, calculate
      number of sensors and motors on controller side
   */
  virtual bool initIntern();

  /** Realizes one to one wiring from robot sensors to controller sensors.
      @param rsensors pointer to array of sensorvalues from robot
      @param rsensornumber number of sensors from robot
      @param csensors pointer to array of sensorvalues for controller
      @param csensornumber number of sensors to controller
      @param noise size of the noise added to the sensors
  */
  virtual bool wireSensorsIntern(const sensor* rsensors, int rsensornumber,
                           sensor* csensors, int csensornumber,
                           double noise);

  /** Realizes one to one wiring from controller motor outputs to robot motors.
      @param rmotors pointer to array of motorvalues for robot
      @param rmotornumber number of robot motors
      @param cmotors pointer to array of motorvalues from controller
      @param cmotornumber number of motorvalues from controller
  */
  virtual bool wireMotorsIntern(motor* rmotors, int rmotornumber,
                          const motor* cmotors, int cmotornumber);


protected:
  double boost;
  matrix::Matrix error;
  matrix::Matrix sens;
};

#endif
