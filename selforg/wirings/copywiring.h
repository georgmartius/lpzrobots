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
#ifndef __COPYWIRING_H
#define __COPYWIRING_H

#include <selforg/abstractwiring.h>
#include <selforg/matrix.h>
#include <list>
#include <vector>

/** Implements a wiring where the motors are copied to several motors
    and the sensors are fusioned.
 */
class CopyWiring: public AbstractWiring {
public:
  typedef std::vector < std::list<int> > Assignment;

  /** constructor
      @param sensor_assignment for each controller sensor a list of robot sensor it is averaged over
      @param motor_assignment for each robot motor a list of controller motors it is averaged over
      @param noise NoiseGenerator that is used for adding noise to sensor values
      @param plotMode see AbstractWiring
   */
  CopyWiring(const Assignment& sensor_assignment, const Assignment& motor_assignment,
             NoiseGenerator* noise, int plotMode=Controller|Robot, const std::string& name = "CopyWiring");

  /** destructor
   */
  virtual ~CopyWiring();

  virtual void reset();

  static Assignment motorFromSensorAssignment(const Assignment& sensor_assignment);

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
  Assignment s_assign;
  Assignment m_assign;
};

#endif
