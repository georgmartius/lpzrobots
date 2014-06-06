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
#include "twoaxisservo.h"
#include "joint.h"
#include <selforg/controller_misc.h>

namespace lpzrobots {

  TwoAxisServo::TwoAxisServo(TwoAxisJoint* joint, double _min1, double _max1, double power1,
                             double _min2, double _max2, double power2,
                             double damp, double integration, double maxVel,
                             double jointLimit, bool minmaxCheck)
    : joint(joint),
      pid1(power1, integration, damp),
      pid2(power2, integration, damp),
      maxVel(maxVel), jointLimit(jointLimit) {

    setMinMax1(_min1,_max1);
    setMinMax2(_min2,_max2);
    assert(min1<max1); assert(min2<max2);
    assert(!minmaxCheck || min1 <= 0); assert(!minmaxCheck || min2 <= 0);
    assert(!minmaxCheck || max1 >= 0); assert(!minmaxCheck || max2 >= 0);
    assert(power1 >=0 && power2 >=0 && damp >=0 && integration >=0);
  }
  TwoAxisServo::~TwoAxisServo(){}

  void TwoAxisServo::set(double pos1, double pos2){
    if(pos1 > 0){
      pos1 *= max1;
    }else{
      pos1 *= -min1;
    }
    pid1.setTargetPosition(pos1);
    // double force1 = pid1.stepWithD(joint->getPosition1(), joint->getPosition1Rate());
    double force1 = pid1.step(joint->getPosition1(), joint->odeHandle.getTime());
    // limit force to 1*KP
    force1 = std::min(pid1.KP, std::max(-pid1.KP,force1));// limit force to 1*KP

    if(pos2 > 0){
      pos2 *= max2;
    }else{
      pos2 *= -min2;
    }
    pid2.setTargetPosition(pos2);
    //      double force2 = pid2.stepWithD(joint->getPosition2(), joint->getPosition2Rate());
    double force2 = pid2.step(joint->getPosition2(), joint->odeHandle.getTime());
    // limit force to 1*KP
    force2 = std::min(pid2.KP, std::max(-pid2.KP,force2));// limit force to 1*KP
    joint->addForces(force1, force2);
    if(maxVel >0 ){
      joint->getPart1()->limitLinearVel(maxVel);
      joint->getPart2()->limitLinearVel(maxVel);
    }
  }


  TwoAxisServoCentered::TwoAxisServoCentered(TwoAxisJoint* joint,
                                             double _min1, double _max1, double power1,
                                             double _min2, double _max2, double power2,
                                             double damp, double integration, double maxVel,
                                             double jointLimit)
    : TwoAxisServo(joint, _min1, _max1, power1, _min2, _max2, power2,
                   damp, integration, maxVel, jointLimit, false){
  }

  TwoAxisServoCentered::~TwoAxisServoCentered(){}

  void TwoAxisServoCentered::set(double pos1, double pos2){
    pos1 = clip(pos1, -1.0, 1.0);
    pos2 = clip(pos2, -1.0, 1.0);
    pos1 = (pos1+1)*(max1-min1)/2 + min1;

    pid1.setTargetPosition(pos1);
    double force1 = pid1.stepNoCutoff(joint->getPosition1(), joint->odeHandle.getTime());
    // limit force to 10*KP
    force1 = clip(force1,-10*pid1.KP, 10*pid1.KP);

    pos2 = (pos2+1)*(max2-min2)/2 + min2;
    pid2.setTargetPosition(pos2);
    double force2 = pid2.stepNoCutoff(joint->getPosition2(), joint->odeHandle.getTime());
    // limit force to 10*KP
    force2 = clip(force2,-10*pid2.KP, 10*pid2.KP);
    joint->addForces(force1, force2);
    if(maxVel >0 ){
      joint->getPart1()->limitLinearVel(maxVel);
      joint->getPart2()->limitLinearVel(maxVel);
    }
  }

  TwoAxisServoVel::TwoAxisServoVel(const OdeHandle& odeHandle,
                                   TwoAxisJoint* joint, double _min1, double _max1, double power1,
                                   double _min2, double _max2, double power2,
                                   double damp, double maxVel, double jointLimit)
    : TwoAxisServoCentered(joint, _min1, _max1, maxVel/2, _min2, _max2, maxVel/2,
                           0, 0, 0, jointLimit),
      // don't wonder! It is correct to give maxVel as a power parameter to the normal servo (PID).
      motor(odeHandle, joint, power1, power2),
      damp(clip(damp,0.0,1.0)), power1(power1), power2(power2)
  {
    motor.init(0,0);
    dummy=0;
  }
  TwoAxisServoVel::~TwoAxisServoVel(){}

  void TwoAxisServoVel::setPower(double _power1, double _power2) {
    motor.setPower(_power1,_power2);
    power1=_power1;
    power2=_power2;
  };
  void TwoAxisServoVel::setPower1(double _power1) {
    power1=_power1;
    motor.setPower(power1,motor.getPower2());

  };
  void TwoAxisServoVel::setPower2(double _power2) {
    power2=_power2;
    motor.setPower(motor.getPower(),power2);
  };

  void TwoAxisServoVel::set(double pos1, double pos2){
    pos1 = clip(pos1, -1.0, 1.0);
    pos2 = clip(pos2, -1.0, 1.0);

    pos1 = (pos1+1.0)*(max1-min1)/2.0 + min1;
    pid1.setTargetPosition(pos1);
    double vel1 = pid1.stepVelocity(joint->getPosition1(), joint->odeHandle.getTime());
    double e1 = fabs(2.0*(pid1.error)/(max1-min1)); // distance from set point

    pos2 = (pos2+1.0)*(max2-min2)/2.0 + min2;
    pid2.setTargetPosition(pos2);
    double vel2 = pid2.stepVelocity(joint->getPosition2(), joint->odeHandle.getTime());
    double e2 = fabs(2.0*(pid2.error)/(max2-min2)); // distance from set point

    motor.set(0, vel1);
    motor.set(1, vel2);
    // calculate power of servo depending on distance from setpoint and damping
    // sigmoid ramping of power for damping < 1
    //      motor.setPower(((1.0-damp)*tanh(e1)+damp) * power1,
    //                     ((1.0-damp)*tanh(e2)+damp) * power2);
    motor.setPower(tanh(e1+damp) * power1,
                   tanh(e2+damp) * power2);

    /*
       // This does not work as well as the amotor, at least for one-axis,
       // so we keep the amotor version
       joint->setParam(dParamVel,  vel1);
       joint->setParam(dParamVel2, vel2);
       joint->setParam(dParamFMax,  tanh(e1+damp) * power1);
       joint->setParam(dParamFMax2, tanh(e2+damp) * power2);
    */

  }

}
