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

#include "oneaxisservo.h"
#include "joint.h"
#include <selforg/controller_misc.h>

namespace lpzrobots {

  OneAxisServo::OneAxisServo(OneAxisJoint* joint, double _min, double _max,
                             double power, double damp, double integration, double maxVel,
                             double jointLimit, bool minmaxCheck)
    : joint(joint), pid(power, integration, damp), maxVel(maxVel), jointLimit(jointLimit) {
    // the joint can also be set later by init()
    setMinMax(_min,_max);
    assert(min<max);
    assert(!minmaxCheck || min < 0);
    assert(!minmaxCheck || max > 0);
    assert(power>=0 && damp >=0 && integration >=0);
  }

  OneAxisServo::~OneAxisServo(){};

  void OneAxisServo::set(double pos){
    pos = clip(pos, -1.0, 1.0);
    if(pos > 0){
      pos *= max;
    }else{
      pos *= -min;
    }
    pid.setTargetPosition(pos);

    double force = pid.step(joint->getPosition1(), joint->odeHandle.getTime());
    force = std::min(pid.KP, std::max(-pid.KP,force));// limit force to 1*KP
    joint->addForce1(force);
    if(maxVel>0){
      joint->getPart1()->limitLinearVel(maxVel);
      joint->getPart2()->limitLinearVel(maxVel);
    }
  }

  OneAxisServoCentered::OneAxisServoCentered(OneAxisJoint* joint, double _min, double _max,
                                             double power, double damp, double integration,
                                             double maxVel, double jointLimit)
    : OneAxisServo(joint, _min, _max, power, damp, integration, maxVel, jointLimit, false){
  }


  void OneAxisServoCentered::set(double pos){
    pos = clip(pos, -1.0, 1.0);
    pos = (pos+1)*(max-min)/2 + min;

    pid.setTargetPosition(pos);
    double force = pid.stepNoCutoff(joint->getPosition1(), joint->odeHandle.getTime());
    force = clip(force,-10*pid.KP, 10*pid.KP); // limit force to 10*KP
    joint->addForce1(force);
    if(maxVel>0){
      joint->getPart1()->limitLinearVel(maxVel);
      joint->getPart2()->limitLinearVel(maxVel);
    }
  }


  OneAxisServoVel::OneAxisServoVel(const OdeHandle& odeHandle,
                                   OneAxisJoint* joint, double _min, double _max,
                                   double power, double damp, double maxVel,
                                   double jointLimit)
    : OneAxisServo(joint, _min, _max, maxVel/2, 0, 0, 0, jointLimit, false),
      // don't wonder! It is correct to give maxVel as a power parameter to the parent.
      motor(odeHandle, joint, power), power(power), damp(clip(damp,0.0,1.0))
  {
    dummy=0;
    motor.init(0,0);
  }

  OneAxisServoVel::~OneAxisServoVel(){}

  void OneAxisServoVel::setPower(double _power) {
    power=_power;
    motor.setPower(power);
  };

  void OneAxisServoVel::set(double pos){
    pos = clip(pos, -1.0, 1.0);
    pos = (pos+1)*(max-min)/2 + min;
    pid.setTargetPosition(pos);
    double vel = pid.stepVelocity(joint->getPosition1(), joint->odeHandle.getTime());
    double e   = fabs(2.0*(pid.error)/(max-min)); // distance from set point
    motor.set(0, vel);
    // calculate power of servo depending on the damping and distance from set point and
    // sigmoid ramping of power for damping < 1
    //      motor.setPower(((1.0-damp)*tanh(e)+damp) * power);
    motor.setPower(tanh(e+damp) * power);

    /*if(maxVel >0 ){ // we limit the maximal velocity (like a air-friction)
    // this hinders the simulation from disintegrating.
    // Not required for velocity servos
    joint->getPart1()->limitLinearVel(5*maxVel);
    joint->getPart2()->limitLinearVel(5*maxVel);
    }
    */
  }

  SliderServoVel::SliderServoVel(const OdeHandle& odeHandle,
                                   OneAxisJoint* joint, double _min, double _max,
                                   double power, double damp, double maxVel,
                                   double jointLimit)
    : OneAxisServo(joint, _min, _max, maxVel/2, 0, 0, 0, jointLimit, false),
      // don't wonder! It is correct to give maxVel as a power parameter to the parent.
      power(power), damp(clip(damp,0.0,1.0))
  {
    dummy=0;
  }

  SliderServoVel::~SliderServoVel(){}

  void SliderServoVel::setPower(double _power) {
    power=_power;
  };

  void SliderServoVel::set(double pos){
    pos = clip(pos, -1.0, 1.0);
    pos = (pos+1)*(max-min)/2 + min;
    pid.setTargetPosition(pos);
    double vel = pid.stepVelocity(joint->getPosition1(), joint->odeHandle.getTime());
    double e   = fabs(2.0*(pid.error)/(max-min)); // distance from set point
    joint->setParam(dParamVel, vel);
    // calculate power of servo depending on the damping and distance from set point and
    // sigmoid ramping of power for damping < 1
    //      (((1.0-damp)*tanh(e)+damp) * power);
    joint->setParam(dParamFMax, tanh(e+damp) * power);

    if(maxVel >0 ){ // we limit the maximal velocity (like a air-friction)
                    // this hinders the simulation from disintegrating.
      joint->getPart1()->limitLinearVel(5*maxVel);
      joint->getPart2()->limitLinearVel(5*maxVel);
    }
  }
}

