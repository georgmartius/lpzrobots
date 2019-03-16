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

#include "locokitmotor.h"

using namespace std;


namespace lpzrobots {

  enum ODEAMOTORAXISREF { GlobalFrame = 0, FirstBody =1, SecondBody = 2};


  LocoKitMotor::LocoKitMotor(const OdeHandle& odeHandle, Joint* joint)
    : motor(0), odeHandle(odeHandle), velocityFactor(1.0), joint(joint),
      initialized(false){

  }

  LocoKitMotor::~LocoKitMotor (){
    if(motor) dJointDestroy(motor);
  }

  void LocoKitMotor::init(Primitive* own, Joint* joint){
    if(initialized) return; // cannot be called twice!
    if(joint) this->joint=joint;
    assert(this->joint!=0);

    const Primitive* p1 = this->joint->getPart1();
    const Primitive* p2 = this->joint->getPart2();
    motor = dJointCreateAMotor(odeHandle.world, 0);
    dJointAttach(motor, p1->getBody(), p2->getBody());
    dJointSetAMotorMode (motor, dAMotorUser);
    setBaseInfo(getBaseInfo().changequantity(SensorMotorInfo::Position));
    initialized=true;
  }

  /** sets the desired speed of all motor.
      @param velocities double array with desired velocities
      @param len length of the given array
      @return number actually set velocities
  */
  int LocoKitMotor::set(const double* velocities, int len){
    int n = min(len, getNumberOfAxes());
    for(int i=0; i< n; i++){
      set(i, velocities[i]);
    }
    return n;
  }

  /** returns the speed (PositionRate) of all axis
      @param velocities double array to fill in the velocities
      @param len length of the given array
      @return number actually returned velocities
  */
  int LocoKitMotor::get(double* velocities, int len) const {
    int n = min(len, getNumberOfAxes());
    for(int i=0; i< n; i++){
      velocities[i] = get(i);
    }
    return n;
  }

/** sets the parameters of the motor
*/
  void LocoKitMotor::setParam(int parameter, double value) {
    assert(initialized);
    dJointSetAMotorParam (motor, parameter, value);
  }

/** gets the parameters of the motor
*/
  double LocoKitMotor::getParam(int parameter){
    assert(initialized);
    return dJointGetAMotorParam(motor, parameter);
  }

  double LocoKitMotor::getPower(){
    assert(initialized);
    return dJointGetAMotorParam(motor, dParamFMax);
  }

  void LocoKitMotor::setVelovityFactor(double factor){
    velocityFactor=factor;
  }


  double LocoKitMotor::getVelovityFactor(double factor){
    return velocityFactor;
  }


  /*******************************************************************************/

  /** Constuct a motor attached to a OneAxisJoint. It will its axis of course.
      @param power The maximum force or torque that the motor will use to achieve the desired velocity.
      This must always be greater than or equal to zero.
      Setting this to zero (the default value) turns off the motor.
  */
  VelocityControlledLocoKitMotor::VelocityControlledLocoKitMotor(const OdeHandle& odeHandle, OneAxisJoint* joint, double power)
  : LocoKitMotor(odeHandle, joint), power(power) {
  }

  void VelocityControlledLocoKitMotor::init(Primitive* own, Joint* _joint){
    if(initialized) return; // cannot be called twice!
    assert(!_joint || dynamic_cast<OneAxisJoint*>(this->joint));
    LocoKitMotor::init(own,_joint);

    dJointSetAMotorNumAxes(motor, 1);
    //    const Axis& a =this->joint->getAxis(0);
    Axis a =this->joint->getAxis(0);
    dJointSetAMotorAxis(motor, 0, FirstBody, a.x(), a.y(), a.z() );
    dJointSetAMotorParam(motor, dParamFMax, power);
    dJointSetAMotorParam(motor, dParamVel, 0);
    setBaseInfo(getBaseInfo().changequantity(SensorMotorInfo::Position));
  }


  /** sets the desired speed of the motor at the given axis.
      @param axisNumber is ignored because have only one axis
      @param velocity Desired motor velocity (this will be an angular or linear velocity).
  */
  void VelocityControlledLocoKitMotor::set(int axisNumber, double velocity) {
    assert(initialized);
    dJointSetAMotorParam(motor, dParamVel, velocity * velocityFactor);
  }

  /** returns the speed (PositionRate) at the given axis, or zero if the axis is out of range
      @param axisNumber is ignored because have only one axis
   */
  double VelocityControlledLocoKitMotor::get(int axisNumber) const {
    assert(initialized);
    double s;
    //joint->getPositionRates(&s);
    joint->getPositions(&s);
    return s;
  }

  /**  sets the maximal force the motor has
   */
  void VelocityControlledLocoKitMotor::setPower(double power){
    dJointSetAMotorParam(motor, dParamFMax, power);
  }


}


