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
#ifndef           MOTOR_H_
#define           MOTOR_H_

#include "globaldata.h"
#include "sensormotorinfoable.h"

namespace lpzrobots {

  /**
      Abstact base class for attachable motors
  */
  class Motor : public virtual SensorMotorInfoAble {
  public:
    Motor() {
    }
    virtual ~Motor() {};

    /** initialises motor with body of robot
    */
    virtual void init(Primitive* own, Joint* joint = 0)=0;

    /// return the dimensionality of this motor
    virtual int getMotorNumber() const =0;

    /** returns a list of motor names  (@see SensorMotorNaming how to change the names) */
    virtual std::list<SensorMotorInfo> getMotorInfos() const {
      return getInfos(getMotorNumber());
    };

    /** performs the actions, This is usually called in
        doInternalStuff() from the robot */
    virtual bool act(GlobalData& globaldata) = 0;

    /** sends the action commands to the motor.
        It returns the number of used values. (should be equal to
        getMotorNumber)
     */
    virtual int set(const motor* values, int length) = 0;
  };
}

#endif /* !MOTOR_H_ */
