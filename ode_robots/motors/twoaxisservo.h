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
#ifndef __SERVO2_H
#define __SERVO2_H

#include "joint.h"
#include "pid.h"
#include "angularmotor.h"
#include <selforg/controller_misc.h>

namespace lpzrobots {

  /** general servo motor for 2 axis joints
   */
  class TwoAxisServo {
  public:
    /** min and max values are understood as travel bounds. Min should be less than 0.*/
  
    TwoAxisServo(TwoAxisJoint* joint, double _min1, double _max1, double power1, 
		 double _min2, double _max2, double power2, 
		 double damp=0.2, double integration=2, double maxVel=10.0, 
		 double jointLimit = 1.3, bool minmaxCheck=true)
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
    virtual ~TwoAxisServo(){}

    /** sets the set point of the servo. 
	Position must be between -1 and 1. It is scaled to fit into min, max
    */
    virtual void set(double pos1, double pos2){
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

    /** returns the position of the servo (joint) of 1. axis in ranges [-1, 1] (scaled by min1, max1)*/
    virtual double get1(){
      double pos =  joint->getPosition1();
      if(pos > 0){
	pos /= max1; 
      }else{
	pos /= -min1;
      }
      return pos;    
    }

    /** returns the position of the servo (joint) of 2. axis in ranges [-1, 1] (scaled by min2, max2)*/
    virtual double get2(){
      double pos =  joint->getPosition2();
      if(pos > 0){
	pos /= max2; 
      }else{
	pos /= -min2;
      }
      return pos;    
    }

    /** returns the positions of the joint in ranges [-1, 1] (scaled by min, max)*/
    void get(double& p1, double& p2){
      p1=get1();
      p2=get2();
    }


    /** adjusts the power of the servo*/
    virtual void setPower(double power1, double power2) { 
      pid1.KP = power1;
      pid2.KP = power2;
    };

    /** returns the power of the servo*/
    virtual void setPower1(double power1) { 
      pid1.KP = power1;
    };

    /** returns the power of the servo*/
    virtual void setPower2(double power2) { 
      pid2.KP = power2;
    };

    /** returns the power of the servo*/
    virtual double getPower1() { 
      return pid1.KP;
    };
    /** returns the power of the servo*/
    virtual double getPower2() { 
      return pid2.KP;
    };

    /** returns the damping of the servo (axis 1) */
    virtual double getDamping1() { 
      return pid1.KD;
    };

    /** returns the damping of the servo (axis 2) */
    virtual double getDamping2() { 
      return pid2.KD;
    };

    virtual TwoAxisJoint* getJoint() {
      return joint;
    }

    /** sets the damping of the servo (axis 1) */
    virtual void setDamping1(double damp) { 
      pid1.KD = damp;
    };

    /** sets the damping of the servo (axis 1) */
    virtual void setDamping2(double damp) { 
      pid2.KD = damp;
    };

    /** sets the damping of the servo (both axis) */
    virtual void setDamping(double _damp) { 
      setDamping1(_damp);
      setDamping2(_damp);
    };

    /** returns the damping of the servo*/
    virtual double& offsetCanceling() { 
      return pid1.KI; 
    };

    virtual void setMinMax1(double _min, double _max){
      min1=_min;
      max1=_max;
      joint->setParam(dParamLoStop, _min  - abs(_min) * (jointLimit-1));
      joint->setParam(dParamHiStop, _max  + abs(_max) * (jointLimit-1));
    }

    virtual void setMinMax2(double _min, double _max){
      min2=_min;
      max2=_max;
      joint->setParam(dParamLoStop2, _min  - abs(_min) * (jointLimit-1));
      joint->setParam(dParamHiStop2, _max  + abs(_max) * (jointLimit-1));
    }

    /** adjusts maximal speed of servo*/
    virtual void setMaxVel(double maxVel) { 
      this->maxVel = maxVel;
    };
    /** adjusts maximal speed of servo*/
    virtual double getMaxVel() { 
      return maxVel;
    };

  
  protected:
    TwoAxisJoint* joint;
    double min1;
    double max1;
    double min2;
    double max2;
    PID pid1;
    PID pid2;
    double maxVel;
    double jointLimit;
  };

  typedef TwoAxisServo UniversalServo;


  /** general servo motor for 2 axis joints with zero position centered
   */
  class TwoAxisServoCentered : public TwoAxisServo {
  public:
    /** min and max values are understood as travel bounds. 
	The zero position is (max-min)/2
    */
    TwoAxisServoCentered(TwoAxisJoint* joint, double _min1, double _max1, double power1, 
			 double _min2, double _max2, double power2, 
			 double damp=0.2, double integration=2, double maxVel=10.0, 
			 double jointLimit = 1.3)
      : TwoAxisServo(joint, _min1, _max1, power1, _min2, _max2, power2,
		     damp, integration, maxVel, jointLimit, false){      
    }
    virtual ~TwoAxisServoCentered(){}

    /** sets the set point of the servo. 
	Position must be between -1 and 1. It is scaled to fit into min, max, 
	however 0 is just in the center of min and max
    */
    virtual void set(double pos1, double pos2){
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

    /** returns the position of the servo (joint) of 1. axis in ranges [-1, 1] 
	(scaled by min1, max1, centered)*/
    virtual double get1(){
      double pos =  joint->getPosition1();      
      return 2*(pos-min1)/(max1-min1) - 1;    
    }


    /** returns the position of the servo (joint) of 2. axis in ranges [-1, 1]
	(scaled by min2, max2, centered)*/
    virtual double get2(){
      double pos =  joint->getPosition2();
      return 2*(pos-min2)/(max2-min2) - 1;    
    }
    
  };

  
  /** general servo motor to achieve position control for 2 axis joints
   *  that internally controls the velocity of the motor (much more stable)
   *  with centered zero position.
   *  The amount of body feeling can be adjusted by the damping parameter
   *   which is understood as a stiffness parameter
   */
  class TwoAxisServoVel : public TwoAxisServoCentered {
  public:
    /** min and max values are understood as travel bounds. 
	The zero position is (max-min)/2
        @param power is the maximal torque the servo can generate
        @param maxVel is understood as a speed parameter of the servo.
        @param damp adjusts the power of the servo in dependence of the distance
         to the set point. This regulates the stiffness and the body feeling
          0: the servo has no power at the set point (maximal body feeling);
          1: is servo has full power at the set point: perfectly damped and stiff.
    */
    TwoAxisServoVel(const OdeHandle& odeHandle, 
		    TwoAxisJoint* joint, double _min1, double _max1, double power1, 
		    double _min2, double _max2, double power2, 
		    double damp=0.05, double maxVel=10.0, double jointLimit = 1.3)
      : TwoAxisServoCentered(joint, _min1, _max1, maxVel/2, _min2, _max2, maxVel/2,
			     0, 0, 0, jointLimit), 
        // don't wonder! It is correct to give maxVel as a power parameter to the normal
        //  servo (PID).
	motor(odeHandle, joint, power1, power2),
        damp(clip(damp,0.0,1.0)), power1(power1), power2(power2)
    {
    }    
    virtual ~TwoAxisServoVel(){}
    
    virtual void setPower(double _power1, double _power2) { 
      motor.setPower(_power1,_power2);
      power1=_power1;
      power2=_power2;
    };
    virtual void setPower1(double _power1) { 
      power1=_power1;
      motor.setPower(power1,motor.getPower2());
    };
    virtual void setPower2(double _power2) { 
      power2=_power2;
      motor.setPower(motor.getPower(),power2);
    };
    virtual double getPower1() { 
      return power1;
    };
    virtual double getPower2() { 
      return power2;
    };

    virtual double getDamping1() { 
      return damp;
    };
    virtual double getDamping2() { 
      return damp;
    };
    virtual void setDamping1(double _damp) { 
      damp = clip(_damp,0.0,1.0);
    };
    virtual void setDamping2(double _damp) { 
      setDamping1(_damp);
    };

    /** offetCanceling does not exist for this type of servo */
    virtual double& offsetCanceling() { 
      dummy=0;
      return dummy;
    };
    
    /** adjusts maximal speed of servo*/
    virtual void setMaxVel(double maxVel) { 
      this->maxVel = maxVel;
      pid1.KP=maxVel/2;
      pid2.KP=maxVel/2;
    };
    /** adjusts maximal speed of servo*/
    virtual double getMaxVel() { 
      return maxVel;      
    };
    
    
    /** sets the set point of the servo. 
	Position must be between -1 and 1. It is scaled to fit into min, max, 
	however 0 is just in the center of min and max
    */
    virtual void set(double pos1, double pos2){
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

    }

  protected:
    AngularMotor2Axis motor;         
    double dummy;
    double damp;
    double power1;
    double power2;    
  };


}
#endif
