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
#ifndef __BRAITENBERG_H
#define __BRAITENBERG_H


#include <stdio.h>
#include <selforg/abstractcontroller.h>

/**
 * simple braitenberg controler type 2 a and b (Aggressive,Cowardly)
 *
 * assumes a linecamera (see LineImgProc), left and right sensor are specified in the constructor
 */
class Braitenberg : public AbstractController {
public:
  enum Type {Aggressive, Cowardly};

  /**
     @param type Braitenberg type
     @param leftsensor index of left light sensor
     @param rightsensor index of right light sensor
     @param leftmotor index of motor of left wheel
     @param rightmotor index of motor of right wheel
   */
  Braitenberg(Type type, int leftsensor, int rightsensor,
              int leftmotor=0, int rightmotor=1)
    : AbstractController("Braitenberg", "0.1"), type(type),
      leftsensor(leftsensor), rightsensor(rightsensor),
      leftmotor(leftmotor), rightmotor(rightmotor) {

    addParameterDef("strength", &strength,0.6);
    addParameterDef("offset", &offset,0.0);
  }

  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0){
    number_sensors=sensornumber;
    number_motors=motornumber;
    assert(sensornumber>=2 && sensornumber>=leftsensor && sensornumber>=rightsensor);
    assert(motornumber>=2  && motornumber>=leftmotor   && motornumber>=rightmotor);
  }

  virtual int getSensorNumber() const {return number_sensors;}

  virtual int getMotorNumber() const {return number_motors;}

  virtual void step(const sensor* sensors, int sensornumber,
                    motor* motors, int motornumber) {
    stepNoLearning(sensors, sensornumber, motors , motornumber);
  }

  virtual void stepNoLearning(const sensor* sensors, int sensornumber,
                              motor* motors, int motornumber){
    switch(type){
    case Aggressive:
      motors[leftmotor] = sensors[rightsensor] * strength + offset;
      motors[rightmotor] = sensors[leftsensor] * strength + offset;
      break;
    case Cowardly:
      motors[leftmotor] = sensors[leftsensor] * strength + offset;
      motors[rightmotor] = sensors[rightsensor] * strength + offset;
      break;
    }
  }


  /********* STORABLE INTERFACE ******/
  /// @see Storable
  virtual bool store(FILE* f) const {
    Configurable::print(f,"");
    return true;
  }

  /// @see Storable
  virtual bool restore(FILE* f) {
    Configurable::parse(f);
    return true;
  }

protected:

  Type type;
  int leftsensor;
  int rightsensor;
  int leftmotor;
  int rightmotor;
  int number_sensors;
  int number_motors;
  paramval strength;
  paramval offset;
};

#endif
