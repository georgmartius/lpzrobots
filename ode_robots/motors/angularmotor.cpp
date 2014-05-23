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

#include <ode-dbl/objects.h>
#include "mathutils.h"
#include "angularmotor.h"
#include "odehandle.h"

using namespace std;


namespace lpzrobots {

  enum ODEAMOTORAXISREF { GlobalFrame = 0, FirstBody =1, SecondBody = 2};


  AngularMotor::AngularMotor(const OdeHandle& odeHandle, Joint* joint)
    : motor(0), odeHandle(odeHandle), velocityFactor(1.0), joint(joint),
      initialized(false){

  }

  AngularMotor::~AngularMotor (){
    if(motor) dJointDestroy(motor);
  }

  void AngularMotor::init(Primitive* own, Joint* joint){
    if(initialized) return; // cannot be called twice!
    if(joint) this->joint=joint;
    assert(this->joint!=0);

    const Primitive* p1 = this->joint->getPart1();
    const Primitive* p2 = this->joint->getPart2();
    motor = dJointCreateAMotor(odeHandle.world, 0);
    dJointAttach(motor, p1->getBody(), p2->getBody());
    dJointSetAMotorMode (motor, dAMotorUser);
    setBaseInfo(getBaseInfo().changequantity(SensorMotorInfo::Velocity));
    initialized=true;
  }

  /** sets the desired speed of all motor.
      @param velocities double array with desired velocities
      @param len length of the given array
      @return number actually set velocities
  */
  int AngularMotor::set(const double* velocities, int len){
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
  int AngularMotor::get(double* velocities, int len) const {
    int n = min(len, getNumberOfAxes());
    for(int i=0; i< n; i++){
      velocities[i] = get(i);
    }
    return n;
  }

/** sets the parameters of the motor
*/
  void AngularMotor::setParam(int parameter, double value) {
    assert(initialized);
    dJointSetAMotorParam (motor, parameter, value);
  }

/** gets the parameters of the motor
*/
  double AngularMotor::getParam(int parameter){
    assert(initialized);
    return dJointGetAMotorParam(motor, parameter);
  }

  double AngularMotor::getPower(){
    assert(initialized);
    return dJointGetAMotorParam(motor, dParamFMax);
  }

  void AngularMotor::setVelovityFactor(double factor){
    velocityFactor=factor;
  }


  double AngularMotor::getVelovityFactor(double factor){
    return velocityFactor;
  }


  /*******************************************************************************/

  /** Constuct a motor attached to a OneAxisJoint. It will its axis of course.
      @param power The maximum force or torque that the motor will use to achieve the desired velocity.
      This must always be greater than or equal to zero.
      Setting this to zero (the default value) turns off the motor.
  */
  AngularMotor1Axis::AngularMotor1Axis(const OdeHandle& odeHandle, OneAxisJoint* joint, double power)
  : AngularMotor(odeHandle, joint), power(power) {
  }

  void AngularMotor1Axis::init(Primitive* own, Joint* _joint){
    if(initialized) return; // cannot be called twice!
    assert(!_joint || dynamic_cast<OneAxisJoint*>(this->joint));
    AngularMotor::init(own,_joint);

    dJointSetAMotorNumAxes(motor, 1);
    //    const Axis& a =this->joint->getAxis(0);
    Axis a =this->joint->getAxis(0);
    dJointSetAMotorAxis(motor, 0, FirstBody, a.x(), a.y(), a.z() );
    dJointSetAMotorParam(motor, dParamFMax, power);
    dJointSetAMotorParam(motor, dParamVel, 0);
  }


  /** sets the desired speed of the motor at the given axis.
      @param axisNumber is ignored because have only one axis
      @param velocity Desired motor velocity (this will be an angular or linear velocity).
  */
  void AngularMotor1Axis::set(int axisNumber, double velocity) {
    assert(initialized);
    dJointSetAMotorParam(motor, dParamVel, velocity * velocityFactor);
  }

  /** returns the speed (PositionRate) at the given axis, or zero if the axis is out of range
      @param axisNumber is ignored because have only one axis
   */
  double AngularMotor1Axis::get(int axisNumber) const {
    assert(initialized);
    double s;
    joint->getPositionRates(&s);
    return s;
  }

  /**  sets the maximal force the motor has
   */
  void AngularMotor1Axis::setPower(double power){
    dJointSetAMotorParam(motor, dParamFMax, power);
  }

  /*******************************************************************************/

  /** Constuct a motor attached to a TwoAxisJoint. It will its two axis of course.
      @param power The maximum force or torque that the motor will use to achieve the desired velocity.
      This must always be greater than or equal to zero.
      Setting this to zero (the default value) turns off the motor.
  */
  AngularMotor2Axis::AngularMotor2Axis(const OdeHandle& odeHandle, TwoAxisJoint* joint,
                                       double power1, double power2)
  : AngularMotor(odeHandle, joint), power1(power1) , power2(power2) {
  }

  void AngularMotor2Axis::init(Primitive* own, Joint* _joint){
    if(initialized) return; // cannot be called twice!
    assert(!_joint || dynamic_cast<TwoAxisJoint*>(this->joint));
    AngularMotor::init(own,_joint);

    dJointSetAMotorNumAxes(motor, 2);
    const Axis& a0 =joint->getAxis(0);
    dJointSetAMotorAxis(motor, 0, FirstBody, a0.x(), a0.y(), a0.z() );
    const Axis& a1 =joint->getAxis(1);
    dJointSetAMotorAxis(motor, 1, SecondBody, a1.x(), a1.y(), a1.z() );

    dJointSetAMotorParam(motor, dParamFMax, power1);
    dJointSetAMotorParam(motor, dParamFMax2, power2);

    dJointSetAMotorParam(motor, dParamVel, 0);
    dJointSetAMotorParam(motor, dParamVel2, 0);
  }

  /** sets the desired speed of the motor at the given axis.
      @param axisNumber either 0 or 1
      @param velocity Desired motor velocity (this will be an angular or linear velocity).
  */
  void AngularMotor2Axis::set(int axisNumber, double velocity){
    assert(initialized);
    int param = dParamVel;
    switch (axisNumber) {
    case 0:
      param = dParamVel;
      break;
    case 1:
      param = dParamVel2;
      break;
    }
    dJointSetAMotorParam(motor, param, velocity * velocityFactor);
  }

  /** returns the speed (PositionRate) at the given axis, or zero if the axis is out of range*/
  double AngularMotor2Axis::get(int axisNumber) const {
    assert(initialized);
    double s[2];
    joint->getPositionRates(s);
    switch (axisNumber) {
    case 0: return s[0];
    case 1: return s[1];
    default:
      return 0;
    }
  }

  /**  sets the maximal force the motor has
   */
  void AngularMotor2Axis::setPower(double power){
    dJointSetAMotorParam(motor, dParamFMax, power);
    dJointSetAMotorParam(motor, dParamFMax2, power);
  }

  void AngularMotor2Axis::setPower(double power1, double power2){
    dJointSetAMotorParam(motor, dParamFMax, power1);
    dJointSetAMotorParam(motor, dParamFMax2, power2);
  }

  double AngularMotor2Axis::getPower2(){
    return dJointGetAMotorParam(motor, dParamFMax2);
  }

  /*******************************************************************************/

  /** Constuct a motor attached to any Joint (not Sliders!).
      The axis have to be provided by the user.
      @param axis list of axis vectors and power. If empty then the motor is disabled.
      Power is the maximum force or torque that the motor will use to achieve the desired velocity.
      This must always be greater than or equal to zero.
      Setting this to zero (the default value) turns off the motor.
  */
  AngularMotorNAxis::AngularMotorNAxis(const OdeHandle& odeHandle, Joint* joint,
                                       std::list<std::pair<double, Axis > > axis)
    : AngularMotor(odeHandle, joint), axis(axis){
  }

  void AngularMotorNAxis::init(Primitive* own, Joint* _joint){
    if(initialized) return; // cannot be called twice!
    AngularMotor::init(own,_joint);

    int len = axis.size();

    dJointSetAMotorNumAxes(motor, len);
    int j=0;
    for(std::list<std::pair<double, Axis > >::iterator i = axis.begin();
        i!= axis.end(); ++i, ++j){
      const Axis& a = (*i).second;
      dJointSetAMotorAxis(motor, j, FirstBody, a.x(), a.y(), a.z() );
      double pwr = (*i).first;
      int param;
      switch (j) {
      case 0:
        param = dParamFMax; break;
      case 1:
        param = dParamFMax2; break;
      case 2:
        param = dParamFMax3; break;
      default:
        continue;
      }
      dJointSetAMotorParam(motor, param, pwr);

      switch (j) {
      case 0:
        param = dParamVel; break;
      case 1:
        param = dParamVel2; break;
      case 2:
        param = dParamVel3; break;
      default:
        continue;
      }
      dJointSetAMotorParam(motor, param, 0);
    }
  }

  /// returns the number of Axis of this Motor
  int AngularMotorNAxis::getNumberOfAxes() const {
    assert(initialized);
    return dJointGetAMotorNumAxes(motor);
  }

  /** sets the desired speed of the motor at the given axis.
      @param axisNumber either 0 or 1
      @param velocity Desired motor velocity (this will be an angular or linear velocity).
  */
  void AngularMotorNAxis::set(int axisNumber, double velocity){
    assert(initialized);
    int param;
    switch (axisNumber) {
    case 0:
      param = dParamVel;
      break;
    case 1:
      param = dParamVel2;
      break;
    case 2:
      param = dParamVel3;
      break;
    default:
      return;
    }
    dJointSetAMotorParam(motor, param, velocity * velocityFactor);
  }

  /** returns the speed (PositionRate) at the given axis, or zero if the axis is out of range*/
  double AngularMotorNAxis::get(int axisNumber) const {
    assert(initialized);
    int param = dParamVel;
    switch (axisNumber) {
    case 0:
      param = dParamVel;
      break;
    case 1:
      param = dParamVel2;
      break;
    case 2:
      param = dParamVel3;
      break;
    }
    return dJointGetAMotorParam(motor, param);
  }

  /**  sets the maximal force the motor has
   */
  void AngularMotorNAxis::setPower(double power){
    int param;
    for(int i=0; i<getNumberOfAxes(); i++) {
      switch (i) {
      case 0:
        param = dParamFMax; break;
      case 1:
        param = dParamFMax2; break;
      case 2:
        param = dParamFMax3; break;
      default:
        continue;
      }
      dJointSetAMotorParam(motor, param, power);
    }
  }

  /*******************************************************************************/

    /** Constuct a motor attached to a BallJoint.
        @param axis1 axis relative to body 1
        @param axis3 axis relative to body 2 (must be perpendicular to axis1
        (the axis 2 is calculated automatically)
        @param power The maximum force or torque that the motor will use to achieve the desired velocity.
        This must always be greater than or equal to zero.
        Setting this to zero (the default value) turns off the motor.
    */
  AngularMotor3AxisEuler::AngularMotor3AxisEuler(const OdeHandle& odeHandle, BallJoint* joint,
                           const Axis& axis1, const Axis& axis3, double power)
  : AngularMotor(odeHandle, joint), axis1(axis1), axis3(axis3), power(power) {
  }

  void AngularMotor3AxisEuler::init(Primitive* own, Joint* joint){
    if(initialized) return; // cannot be called twice!
    assert(!joint || dynamic_cast<BallJoint*>(this->joint));
    AngularMotor::init(own,joint);

    dJointSetAMotorNumAxes(motor, 3);
    dJointSetAMotorMode (motor, dAMotorEuler);

    dJointSetAMotorAxis(motor, 0, FirstBody, axis1.x(), axis1.y(), axis1.z() );
    dJointSetAMotorAxis(motor, 2, SecondBody, axis3.x(), axis3.y(), axis3.z() );


    dJointSetAMotorParam(motor, dParamFMax, power);
    dJointSetAMotorParam(motor, dParamFMax2, power);
    dJointSetAMotorParam(motor, dParamFMax3, power);
  }

  /** sets the desired speed of the motor at the given axis.
      @param axisNumber either 0 or 1
      @param velocity Desired motor velocity (this will be an angular or linear velocity).
  */
  void AngularMotor3AxisEuler::set(int axisNumber, double velocity){
    assert(initialized);
    int param;
    switch (axisNumber) {
    case 0:
      param = dParamVel;
      break;
    case 1:
      param = dParamVel2;
      break;
    case 2:
      param = dParamVel3;
      break;
    default:
      return;
    }
    dJointSetAMotorParam(motor, param, velocity * velocityFactor);
  }

  /** returns the speed (PositionRate) at the given axis, or zero if the axis is out of range*/
  double AngularMotor3AxisEuler::get(int axisNumber) const{
    switch (axisNumber) {
    case 0:
      return dJointGetAMotorAngleRate(motor, 0);
    case 1:
      return dJointGetAMotorAngleRate(motor, 1);
    case 2:
      return dJointGetAMotorAngleRate(motor, 2);
    default:
      return 0;
    }
  }

  /**  sets the maximal force the motor has
   */
  void AngularMotor3AxisEuler::setPower(double power){
    dJointSetAMotorParam(motor, dParamFMax, power);
    dJointSetAMotorParam(motor, dParamFMax2, power);
    dJointSetAMotorParam(motor, dParamFMax3, power);
  }


}


