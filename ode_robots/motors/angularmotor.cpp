/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
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
 ***************************************************************************
 *                                                                         *
 *  DESCRIPTION                                                            *
 *                                                                         *
 *   $Log$
 *   Revision 1.10  2010-07-05 13:10:43  martius
 *   Axis have been in global coordinates and not relative to one of the bodies!
 *   This could not work.
 *   Expecially problematic with twoaxis joints and the XServoVel implementation.
 *
 *   Revision 1.9  2010/03/09 11:53:41  martius
 *   renamed globally ode to ode-dbl
 *
 *   Revision 1.8  2009/08/10 14:46:41  der
 *   power() functions removed because references are bad vor velocity servo
 *   setPower() functions added
 *
 *   Revision 1.7  2009/05/11 15:43:22  martius
 *   new velocity controlling servo motors
 *
 *   Revision 1.6  2008/05/07 16:45:51  martius
 *   code cosmetics and documentation
 *
 *   Revision 1.5  2007/07/03 13:10:13  martius
 *   initialization of variables
 *
 *   Revision 1.4  2006/09/21 16:15:06  der
 *   *** empty log message ***
 *
 *   Revision 1.3  2006/07/20 17:19:43  martius
 *   removed using namespace std from matrix.h
 *
 *   Revision 1.2  2006/07/14 12:23:32  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.4  2006/06/29 16:39:55  robot3
 *   -you can now see bounding shapes if you type ./start -drawboundings
 *   -includes cleared up
 *   -abstractobstacle and abstractground have now .cpp-files
 *
 *   Revision 1.1.2.3  2006/02/23 18:05:30  martius
 *   setPower (on all axis the same)
 *
 *   Revision 1.1.2.2  2006/02/07 15:51:56  martius
 *   axis, setpower
 *
 *   Revision 1.1.2.1  2005/12/21 15:38:32  martius
 *   Angular motors nicely wrapped
 *
 *   Revision 1.1.2.1  2005/12/06 17:38:21  martius
 *   *** empty log message ***
 *
 *                                                                 *
 ***************************************************************************/

#include <ode-dbl/objects.h>
#include "mathutils.h"
#include "angularmotor.h"
#include "odehandle.h"

using namespace std;


namespace lpzrobots {

  enum ODEAMOTORAXISREF { GlobalFrame = 0, FirstBody =1, SecondBody = 2};


  AngularMotor::AngularMotor(const OdeHandle& odeHandle, Joint* joint){
    const Primitive* p1 = joint->getPart1();
    const Primitive* p2 = joint->getPart2();
    motor = dJointCreateAMotor(odeHandle.world, 0);
    dJointAttach(motor, p1->getBody(), p2->getBody());
    dJointSetAMotorMode (motor, dAMotorUser);
  }
  
  AngularMotor::~AngularMotor (){
    dJointDestroy(motor);
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
  int AngularMotor::get(double* velocities, int len){
    int n = min(len, getNumberOfAxes());
    for(int i=0; i< n; i++){
      velocities[i] = get(i);
    }
    return n;
  }

/** sets the parameters of the motor
*/
  void AngularMotor::setParam(int parameter, double value) {
	dJointSetAMotorParam (motor, parameter, value);
  }

/** gets the parameters of the motor
*/
  double AngularMotor::getParam(int parameter){
     return dJointGetAMotorParam(motor, parameter);
  }

  double AngularMotor::getPower(){
    return dJointGetAMotorParam(motor, dParamFMax);       
  }

  /*******************************************************************************/

  /** Constuct a motor attached to a OneAxisJoint. It will its axis of course.
      @param power The maximum force or torque that the motor will use to achieve the desired velocity. 
      This must always be greater than or equal to zero. 
      Setting this to zero (the default value) turns off the motor.      
  */
  AngularMotor1Axis::AngularMotor1Axis(const OdeHandle& odeHandle, OneAxisJoint* joint, double power)
    : AngularMotor(odeHandle, joint), joint(joint) {
    dJointSetAMotorNumAxes(motor, 1);
    const Axis& a =joint->getAxis1();
    dJointSetAMotorAxis(motor, 0, FirstBody, a.x(), a.y(), a.z() );
    dJointSetAMotorParam(motor, dParamFMax, power); 
    dJointSetAMotorParam(motor, dParamVel, 0); 
  }
  
  
  /** sets the desired speed of the motor at the given axis.
      @param axisNumber is ignored because have only one axis
      @param velocity Desired motor velocity (this will be an angular or linear velocity).
  */
  void AngularMotor1Axis::set(int axisNumber, double velocity){
    dJointSetAMotorParam(motor, dParamVel, velocity); 
  }

  /** returns the speed (PositionRate) at the given axis, or zero if the axis is out of range
      @param axisNumber is ignored because have only one axis
   */
  double AngularMotor1Axis::get(int axisNumber){
    return joint->getPosition1Rate(); 
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
    : AngularMotor(odeHandle, joint), joint(joint) {

    dJointSetAMotorNumAxes(motor, 2);
    const Axis& a0 =joint->getAxis1();
    dJointSetAMotorAxis(motor, 0, FirstBody, a0.x(), a0.y(), a0.z() );
    const Axis& a1 =joint->getAxis2();
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
    int param = dParamVel;
    switch (axisNumber) {
    case 0:
      param = dParamVel; 
      break;
    case 1:
      param = dParamVel2; 
      break;
    }
    dJointSetAMotorParam(motor, param, velocity); 
  }

  /** returns the speed (PositionRate) at the given axis, or zero if the axis is out of range*/
  double AngularMotor2Axis::get(int axisNumber){
    switch (axisNumber) {
    case 0:
      return joint->getPosition1Rate(); 
    case 1:
      return joint->getPosition2Rate(); 
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
    : AngularMotor(odeHandle, joint), joint(joint) {
    
    int len = axis.size();
    
    dJointSetAMotorNumAxes(motor, len);
    int j=0;
    for(std::list<std::pair<double, Axis > >::iterator i = axis.begin(); 
	i!= axis.end(); i++, j++){
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
  int AngularMotorNAxis::getNumberOfAxes(){
    return dJointGetAMotorNumAxes(motor);
  }

  /** sets the desired speed of the motor at the given axis.       
      @param axisNumber either 0 or 1
      @param velocity Desired motor velocity (this will be an angular or linear velocity).
  */
  void AngularMotorNAxis::set(int axisNumber, double velocity){
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
    dJointSetAMotorParam(motor, param, velocity); 
  }

  /** returns the speed (PositionRate) at the given axis, or zero if the axis is out of range*/
  double AngularMotorNAxis::get(int axisNumber){    
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
      : AngularMotor(odeHandle, joint), joint(joint) {
          
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
    dJointSetAMotorParam(motor, param, velocity); 
  }

  /** returns the speed (PositionRate) at the given axis, or zero if the axis is out of range*/
  double AngularMotor3AxisEuler::get(int axisNumber){    
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


