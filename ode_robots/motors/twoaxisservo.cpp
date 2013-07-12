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
    assert(joint);
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


  double TwoAxisServo::get1(){
    double pos =  joint->getPosition1();
    if(pos > 0){
      pos /= max1;
    }else{
      pos /= -min1;
    }
    return pos;
  }


  double TwoAxisServo::get2(){
    double pos =  joint->getPosition2();
    if(pos > 0){
      pos /= max2;
    }else{
      pos /= -min2;
    }
    return pos;
  }


  void TwoAxisServo::get(double& p1, double& p2){
    p1=get1();
    p2=get2();
  }



  void TwoAxisServo::setPower(double power1, double power2) {
    pid1.KP = power1;
    pid2.KP = power2;
  };


  void TwoAxisServo::setPower1(double power1) {
    pid1.KP = power1;
  };


  void TwoAxisServo::setPower2(double power2) {
    pid2.KP = power2;
  };


  double TwoAxisServo::getPower1() {
    return pid1.KP;
  };

  double TwoAxisServo::getPower2() {
    return pid2.KP;
  };


  double TwoAxisServo::getDamping1() {
    return pid1.KD;
  };


  double TwoAxisServo::getDamping2() {
    return pid2.KD;
  };

  TwoAxisJoint* TwoAxisServo::getJoint() {
    return joint;
  }


  void TwoAxisServo::setDamping1(double damp) {
    pid1.KD = damp;
  };


  void TwoAxisServo::setDamping2(double damp) {
    pid2.KD = damp;
  };


  void TwoAxisServo::setDamping(double _damp) {
    setDamping1(_damp);
    setDamping2(_damp);
  };


  double& TwoAxisServo::offsetCanceling() {
    return pid1.KI;
  };

  void TwoAxisServo::setMinMax1(double _min, double _max){
    min1=_min;
    max1=_max;
    joint->setParam(dParamLoStop, _min  - abs(_min) * (jointLimit-1));
    joint->setParam(dParamHiStop, _max  + abs(_max) * (jointLimit-1));
  }

  void TwoAxisServo::setMinMax2(double _min, double _max){
    min2=_min;
    max2=_max;
    joint->setParam(dParamLoStop2, _min  - abs(_min) * (jointLimit-1));
    joint->setParam(dParamHiStop2, _max  + abs(_max) * (jointLimit-1));
  }


  void TwoAxisServo::setMaxVel(double maxVel) {
    this->maxVel = maxVel;
  };

  double TwoAxisServo::getMaxVel() {
    return maxVel;
  };

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

  double TwoAxisServoCentered::get1(){
    double pos =  joint->getPosition1();
    return 2*(pos-min1)/(max1-min1) - 1;
  }


  double TwoAxisServoCentered::get2(){
    double pos =  joint->getPosition2();
    return 2*(pos-min2)/(max2-min2) - 1;
  }


  TwoAxisServoVel::TwoAxisServoVel(const OdeHandle& odeHandle,
                                   TwoAxisJoint* joint, double _min1, double _max1, double power1,
                                   double _min2, double _max2, double power2,
                                   double damp, double maxVel, double jointLimit)
    : TwoAxisServoCentered(joint, _min1, _max1, maxVel/2, _min2, _max2, maxVel/2,
                           0, 0, 0, jointLimit),
      // don't wonder! It is correct to give maxVel as a power parameter to the normal servo (PID).
      damp(clip(damp,0.0,1.0)), power1(power1), power2(power2)
  {
  }
  TwoAxisServoVel::~TwoAxisServoVel(){}

  void TwoAxisServoVel::setPower(double _power1, double _power2) {
    power1=_power1;
    power2=_power2;
  };
  void TwoAxisServoVel::setPower1(double _power1) {
    power1=_power1;
  };
  void TwoAxisServoVel::setPower2(double _power2) {
    power2=_power2;
  };
  double TwoAxisServoVel::getPower1() {
    return power1;
  };
  double TwoAxisServoVel::getPower2() {
    return power2;
  };

  double TwoAxisServoVel::getDamping1() {
    return damp;
  };
  double TwoAxisServoVel::getDamping2() {
    return damp;
  };
  void TwoAxisServoVel::setDamping1(double _damp) {
    damp = clip(_damp,0.0,1.0);
  };
  void TwoAxisServoVel::setDamping2(double _damp) {
    setDamping1(_damp);
  };


  double& TwoAxisServoVel::offsetCanceling() {
    dummy=0;
    return dummy;
  };


  void TwoAxisServoVel::setMaxVel(double maxVel) {
    this->maxVel = maxVel;
    pid1.KP=maxVel/2;
    pid2.KP=maxVel/2;
  };

  double TwoAxisServoVel::getMaxVel() {
    return maxVel;
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

    joint->setParam(dParamVel,  vel1);
    joint->setParam(dParamVel2, vel2);

    // calculate power of servo depending on distance from setpoint and damping
    // sigmoid ramping of power for damping < 1
    //      (((1.0-damp)*tanh(e1)+damp) * power1,
    //       ((1.0-damp)*tanh(e2)+damp) * power2);
    joint->setParam(dParamFMax,  tanh(e1+damp) * power1);
    joint->setParam(dParamFMax2, tanh(e2+damp) * power2);

    if(maxVel >0 ){ // we limit the maximal velocity (like a air-friction)
                    // this hinders the simulation from disintegrating.
      joint->getPart1()->limitLinearVel(5*maxVel);
      joint->getPart2()->limitLinearVel(5*maxVel);
    }


  }

}
