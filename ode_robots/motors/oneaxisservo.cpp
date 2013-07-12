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
    assert(joint);
    setMinMax(_min,_max);
    assert(min<max);
    assert(!minmaxCheck || min <= 0);
    assert(!minmaxCheck || max >= 0);
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

  double OneAxisServo::get(){
    double pos =  joint->getPosition1();
    if(pos > 0){
      pos /= max;
    }else{
      pos /= -min;
    }
    return pos;
  }

  void OneAxisServo::setMinMax(double _min, double _max){
    min=_min;
    max=_max;
    joint->setParam(dParamLoStop, min  - abs(min) * (jointLimit-1));
    joint->setParam(dParamHiStop, max  + abs(max) * (jointLimit-1));
  }

  void OneAxisServo::setPower(double power) {
    pid.KP = power;
  };

  double OneAxisServo::getPower() {
    return pid.KP;
  };

  double OneAxisServo::getDamping() {
    return pid.KD;
  }

  void OneAxisServo::setDamping(double damp) {
    pid.KD = damp;
  };

  double& OneAxisServo::offsetCanceling() {
    return pid.KI;
  };

  void OneAxisServo::setMaxVel(double maxVel) {
    this->maxVel = maxVel;
  };

  double  OneAxisServo::getMaxVel() {
    return maxVel;
  };


  OneAxisServoCentered::OneAxisServoCentered(OneAxisJoint* joint, double _min, double _max,
                                             double power, double damp, double integration,
                                             double maxVel, double jointLimit)
    : OneAxisServo(joint, _min, _max, power, damp, integration, maxVel, jointLimit, false){
  }

  OneAxisServoCentered::~OneAxisServoCentered(){}

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
  double OneAxisServoCentered::get(){
    double pos =  joint->getPosition1();

    return 2*(pos-min)/(max-min) - 1;
  }

  OneAxisServoVel::OneAxisServoVel(const OdeHandle& odeHandle,
                                   OneAxisJoint* joint, double _min, double _max,
                                   double power, double damp, double maxVel,
                                   double jointLimit)
    : OneAxisServo(joint, _min, _max, maxVel/2, 0, 0, 0, jointLimit, false),
      // don't wonder! It is correct to give maxVel as a power parameter to the parent.
      /*motor(odeHandle, joint, power),*/ power(power), damp(clip(damp,0.0,1.0))
  {

  }

  OneAxisServoVel::~OneAxisServoVel(){}

  void OneAxisServoVel::setPower(double _power) {
    power=_power;
    // motor.setPower(power);
  };

  double OneAxisServoVel::getPower() {
    return power;
  };

  double OneAxisServoVel::getDamping() {
    return damp;
  };

  void OneAxisServoVel::setDamping(double _damp) {
    damp = clip(_damp,0.0,1.0);
  };

  double& OneAxisServoVel::offsetCanceling() {
    dummy=0;
    return dummy;
  };

  void OneAxisServoVel::setMaxVel(double maxVel) {
    this->maxVel = maxVel;
    pid.KP=maxVel/2;
  };

  double OneAxisServoVel::getMaxVel() {
    return maxVel;
  };

  void OneAxisServoVel::set(double pos){
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

  double OneAxisServoVel::get(){
    double pos =  joint->getPosition1();
    return 2*(pos-min)/(max-min) - 1;
  }

}

