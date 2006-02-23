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
 *                                                                         *
 *   $Log$
 *   Revision 1.1.2.5  2006-02-23 18:05:30  martius
 *   setPower (on all axis the same)
 *
 *   Revision 1.1.2.4  2006/02/07 15:51:56  martius
 *   axis, setpower
 *
 *   Revision 1.1.2.3  2006/01/31 15:43:47  martius
 *   *** empty log message ***
 *
 *   Revision 1.1.2.2  2006/01/03 10:20:16  fhesse
 *   methods of AngularMotor1Axis public now
 *
 *   Revision 1.1.2.1  2005/12/21 15:38:12  martius
 *   angular motors nicely wrapped
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __ANGULARMOTOR_H
#define __ANGULARMOTOR_H

#include <list>
#include "joint.h"

namespace lpzrobots {

  /** Abstract angular motor class. This is a wrapper for ODE's AMotor.
   */
  class AngularMotor {
  public:
    /// creates a AMotor attached to the same bodies as the given joint.
    AngularMotor(const OdeHandle& odeHandle, Joint* joint);

    virtual ~AngularMotor (){}

    /// returns the number of Axis of this Motor
    virtual int getNumberOfAxes() = 0;

    /** sets the desired speed of the motor at the given axis.       
	@param velocity Desired motor velocity (this will be an angular or linear velocity).
    */
    virtual void set(int axisNumber, double velocity) = 0;
    /** returns the speed (PositionRate) at the given axis, or zero if the axis is out of range*/
    virtual double get(int axisNumber) = 0;

    /**  sets the maximal force the motor has
     */
    virtual void setPower(double power) = 0;


    /** sets the desired speed of all motor.
	@param velocities double array with desired velocities
	@param len length of the given array
	@return number actually returned velocities
    */
    virtual int set(const double* velocities, int len);
    /** returns the speed (PositionRate) of all axis
	@param velocities double array to fill in the velocities
	@param len length of the given array
	@return number actually returned velocities
    */
    virtual int get(double* velocities, int len);
  
  protected:
    dJointID motor;  
  };


  /// Angular motor for OneAxisJoints
  class AngularMotor1Axis : public AngularMotor {
  public:
    /** Constuct a motor attached to a OneAxisJoint. It will its axis of course.
	@param power The maximum force or torque that the motor will use to achieve the desired velocity. 
	This must always be greater than or equal to zero. 
	Setting this to zero (the default value) turns off the motor.      
    */
    AngularMotor1Axis(const OdeHandle& odeHandle, OneAxisJoint* joint, double power);
    virtual ~AngularMotor1Axis() {}
    
    /// returns the number of Axis of this Motor
    virtual int getNumberOfAxes() { return 1; };

    /** sets the desired speed of the motor at the given axis.       
	@param axisNumber is ignored because have only one axis
	@param velocity Desired motor velocity (this will be an angular or linear velocity).
    */
    virtual void set(int axisNumber, double velocity);
    /** returns the speed (PositionRate) at the given axis, or zero if the axis is out of range
	@param axisNumber is ignored because have only one axis
     */
    virtual double get(int axisNumber) ;    

    /**  sets the maximal force the motor has
     */
    virtual void setPower(double power);
    
  protected:
    OneAxisJoint* joint;  
  };

  /// Angular motor for TwoAxisJoints
  class AngularMotor2Axis : public AngularMotor {
  public:
    /** Constuct a motor attached to a TwoAxisJoint. It will its two axis of course.
	@param power The maximum force or torque that the motor will use to achieve the desired velocity. 
	This must always be greater than or equal to zero. 
	Setting this to zero (the default value) turns off the motor.      
    */
    AngularMotor2Axis(const OdeHandle& odeHandle, TwoAxisJoint* joint, double power1, double power2);
    virtual ~AngularMotor2Axis() {}

    /// returns the number of Axis of this Motor
    virtual int getNumberOfAxes() { return 2; };

    /** sets the desired speed of the motor at the given axis.       
	@param axisNumber either 0 or 1
	@param velocity Desired motor velocity (this will be an angular or linear velocity).
    */
    virtual void set(int axisNumber, double velocity);
    /** returns the speed (PositionRate) at the given axis, or zero if the axis is out of range*/
    virtual double get(int axisNumber) ;    
    
    /**  sets the maximal force the motor has
     */
    virtual void setPower(double power);

  protected:
    TwoAxisJoint* joint;      
  };


  /// Angular motor for Ball Joints with Euler control
  class AngularMotor3AxisEuler : public AngularMotor {
  public:
    /** Constuct a motor attached to a BallJoint. 
	@param axis1 axis relative to body 1
	@param axis3 axis relative to body 2 (must be perpendicular to axis1 
	(the axis 2 is calculated automatically) 	
	@param power The maximum force or torque that the motor will use to achieve the desired velocity. 
	This must always be greater than or equal to zero. 
	Setting this to zero (the default value) turns off the motor.      
    */
    AngularMotor3AxisEuler(const OdeHandle& odeHandle, BallJoint* joint, 
			   const Axis& axis1, const Axis& axis3, double power);
    
    /// returns the number of Axis of this Motor
    virtual int getNumberOfAxes() { return 3; };

    /** sets the desired speed of the motor at the given axis.       
	@param axisNumber either 0 or 1
	@param velocity Desired motor velocity (this will be an angular or linear velocity).
    */
    virtual void set(int axisNumber, double velocity);
    /** returns the speed (PositionRate) at the given axis, or zero if the axis is out of range*/
    virtual double get(int axisNumber) ;    
    
    /**  sets the maximal force the motor has
     */
    virtual void setPower(double power);

  protected:
    BallJoint* joint;      
  };

  /// Angular motor for arbitrary Joints with custom axis (up to 3)
  class AngularMotorNAxis : public AngularMotor {
  public:
    /** Constuct a motor attached to any Joint (not Sliders!). 
	The axis have to be provided by the user.	
	@axis list of axis vector and power If empty then it motor is disabled. 
	Power is the maximum force or torque that the motor will use to achieve the desired velocity. 
	This must always be greater than or equal to zero. 
	Setting this to zero (the default value) turns off the motor.      
    */
    AngularMotorNAxis(const OdeHandle& odeHandle, Joint* joint, 
		      std::list<std::pair<double, Axis > > axis);

    virtual ~AngularMotorNAxis() {}
    
    /// returns the number of Axis of this Motor
    virtual int getNumberOfAxes();

    /** sets the desired speed of the motor at the given axis.       
	@param velocity Desired motor velocity (this will be an angular or linear velocity).
    */
    virtual void set(int axisNumber, double velocity);
    /** returns the speed (PositionRate) at the given axis, or zero if the axis is out of range
	The problem is, that we don't have actual information available. 
	So we return the last set position!.
     */
    virtual double get(int axisNumber) ;    

    /**
       sets the maximal force the motor has
     */
    virtual void setPower(double power);
    
  protected:
    Joint* joint; 
  };



}
#endif
