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
#ifndef __ABSTRACTROBOT_H
#define __ABSTRACTROBOT_H

#include <vector>
#include <list>
#include <string>

#include "trackable.h"
#include "configurable.h"
#include "position.h"
#include "sensormotorinfo.h"

/**
 * Abstract class (interface) for robot in general
 *
 *
 */
class AbstractRobot : public Trackable, public Configurable {
public:
  typedef double sensor;
  typedef double motor;

  /**
   * Constructor
   * @param name name of the robot
   * @param revision revision number of the file (Hint: use CVS variable \verbatim $ID$ \endverbatim )
   */
  AbstractRobot(const std::string& name="abstractRobot", const std::string& revision = "$ID$")
    : Configurable(name, revision) {
  };

  virtual ~AbstractRobot(){}

  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1]
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  virtual int getSensors(sensor* sensors, int sensornumber)=0;

  /** sets actual motorcommands
      @param motors motors scaled to [-1,1]
      @param motornumber length of the motor array
  */
  virtual void setMotors(const motor* motors, int motornumber)=0;

  /** returns number of sensors
  */
  virtual int getSensorNumber()=0;

  /** returns number of motors
  */
  virtual int getMotorNumber()=0;

  virtual std::string getTrackableName() const {return getName();}

  /** returns the information for the sensors.
      The following relation has to hold: getSensorNames().size() == getSensorNumber()
   */
  virtual std::list<SensorMotorInfo> getSensorInfos()
  { return std::list<SensorMotorInfo>(getSensorNumber());};

  /** returns the information for the motors.
      The following relation has to hold: getMotorNames().size() == getMotorNumber()
   */
  virtual std::list<SensorMotorInfo> getMotorInfos()
  { return std::list<SensorMotorInfo>(getMotorNumber());};


};

#endif

