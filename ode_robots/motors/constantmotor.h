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
#ifndef           CONSTANTMOTOR_H_
#define           CONSTANTMOTOR_H_

#include <memory>

#include "motor.h"


namespace lpzrobots {

  /**
      Wrapper for Motor to have a constant set value (resulting in getMotorNumber()=0)
  */
  class ConstantMotor : public Motor {
  public:
    /// motor to wrap and constant values (for first and second motor value, all others = value1)
    ConstantMotor(std::shared_ptr<Motor> motor, double value1=0.0, double value2=0.0)
      : motor(motor), value1(value1), value2(value2)
    { }

    virtual void init(Primitive* own, Joint* joint = 0) override {
      motor->init(own, joint);
    }

    virtual int getMotorNumber() const override { return 0; };

    virtual bool act(GlobalData& globaldata) {
      int len = motor->getMotorNumber();
      double* dat = new double[len];
      std::fill_n( dat, len, value1);
      if(len>1) dat[1]=value2;
      motor->set(dat,len);
      return true;
    };

    // does nothing. the actions are done in act
    virtual int set(const motor* values, int length) { return 0;};
  protected:
    std::shared_ptr<Motor> motor;
    double value1;
    double value2;
  };

}

#endif /* !MOTOR_H_ */
