/***************************************************************************
 *   Copyright (C) 2005-2012 LpzRobots development team                    *
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
#ifndef __REMOTECONTROLLED_H
#define __REMOTECONTROLLED_H

#include "abstractcontroller.h"

#include <selforg/matrix.h>

/**
 * Controller that is explicity controlled remotely (no own intelligence).
 * Call remoteControl() each step to set the motor values
 */
class RemoteControlled : public AbstractController {
public:
  RemoteControlled()
    : AbstractController("RemoteControlled", "1.0")
  {
  }

  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0){
    number_sensors = sensornumber;
    number_motors  = motornumber;
    x.set(number_sensors,1);
    y.set(number_motors,1);
  }

  virtual int getSensorNumber() const { return number_sensors;};

  virtual int getMotorNumber() const { return number_motors; };

  virtual void step(const sensor* sensors, int sensornumber,
                    motor* motors, int motornumber){
    stepNoLearning(sensors,sensornumber, motors, motornumber);
  }

  virtual void stepNoLearning(const sensor* sensors, int number_sensors,
                              motor* motors, int number_motors){
    assert(this->number_motors<=number_motors);
    assert(this->number_sensors<=number_sensors);
    x.set(sensors);
    y.convertToBuffer(motors, this->number_motors);
  }

  virtual void remoteControl(const matrix::Matrix& motors){
    assert(motors.getN() == 1 && motors.getM() == y.getM());
    y=motors;
  }

  virtual matrix::Matrix getLastSensorValues(){
    return x;
  }

  virtual bool store(FILE* f) const { return true;};

  /** loads the object from the given file stream (binary).
  */
  virtual bool restore(FILE* f){ return true; };


protected:
  unsigned short number_sensors;
  unsigned short number_motors;
  matrix::Matrix x;        // current sensor value vector
  matrix::Matrix y;        // current motor value vector
};

#endif
