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
#ifndef __SERVO1_H
#define __SERVO1_H

#include "joint.h"
#include "pid.h"
#include "angularmotor.h"
#include <selforg/controller_misc.h>

namespace lpzrobots {

  /** general servo motor to achieve position control 
   */
  class OneAxisServo {
  public:
    /** min and max values are understood as travel bounds. Min should be less than 0.*/
  
    OneAxisServo(OneAxisJoint* joint, double _min, double _max, 
		 double power, double damp=0.2, double integration=2, double maxVel=10.0,
		 double jointLimit = 1.3, bool minmaxCheck = true)
      : joint(joint), pid(power, integration, damp), maxVel(maxVel), jointLimit(jointLimit) { 
      assert(joint); 
      setMinMax(_min,_max);
      assert(min<max);     
      assert(!minmaxCheck || min <= 0);
      assert(!minmaxCheck || max >= 0);
      assert(power>=0 && damp >=0 && integration >=0);
    }

    virtual ~OneAxisServo(){}

    /** sets the set point of the servo. 
	Position must be between -1 and 1. It is scaled to fit into min, max
    */
    virtual void set(double pos){
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

    /** returns the position of the slider in ranges [-1, 1] (scaled by min, max)*/
    virtual double get(){
      double pos =  joint->getPosition1();
      if(pos > 0){
	pos /= max; 
      }else{
	pos /= -min;
      }
      return pos;    
    }

    virtual void setMinMax(double _min, double _max){
      min=_min;
      max=_max;
      joint->setParam(dParamLoStop, min  - abs(min) * (jointLimit-1));
      joint->setParam(dParamHiStop, max  + abs(max) * (jointLimit-1));
    }

    /** adjusts the power of the servo*/
    virtual void setPower(double power) { 
      pid.KP = power;
    };

    /** returns the power of the servo*/
    virtual double getPower() { 
      return pid.KP;
    };

    /** returns the damping of the servo*/
    virtual double getDamping() { 
      return pid.KD;
    }
    /** sets the damping of the servo*/
    virtual void setDamping(double damp) { 
      pid.KD = damp;
    };

    /** returns the integration term of the PID controller of the servo*/
    virtual double& offsetCanceling() { 
      return pid.KI;
    };
    
    /** adjusts maximal speed of servo*/
    virtual void setMaxVel(double maxVel) { 
      this->maxVel = maxVel;
    };
    /** adjusts maximal speed of servo*/
    virtual double getMaxVel() { 
      return maxVel;
    };

  
  protected:
    OneAxisJoint* joint;
    double min;
    double max;
    PID pid;
    double maxVel;
    double jointLimit; ///< joint limit with respect to servo limit
  };

  typedef OneAxisServo SliderServo;
  typedef OneAxisServo HingeServo;
  typedef OneAxisServo Hinge2Servo;



  /** general servo motor to achieve position control with zero position centered
   */
  class OneAxisServoCentered : public OneAxisServo {
  public:
    /** min and max values are understood as travel bounds. 
	The zero position is (max-min)/2
    */
    OneAxisServoCentered(OneAxisJoint* joint, double _min, double _max, 
			 double power, double damp=0.2, double integration=2, 
			 double maxVel=10.0, double jointLimit = 1.3)
      : OneAxisServo(joint, _min, _max, power, damp, integration, maxVel, jointLimit, false){      
    }
    virtual ~OneAxisServoCentered(){}

    /** sets the set point of the servo. 
	Position must be between -1 and 1. It is scaled to fit into min, max, 
	however 0 is just in the center of min and max
    */
    virtual void set(double pos){
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
    /** returns the position of the slider in ranges [-1, 1] (scaled by min, max, centered)*/
    virtual double get(){
      double pos =  joint->getPosition1();
      
      return 2*(pos-min)/(max-min) - 1;    
    }    
    
  };

  /** general servo motor to achieve position control 
   *  that that internally controls the velocity of the motor (much more stable)
   *  with centered zero position.
   *  The amount of body feeling can be adjusted by the damping parameter
   */
  class OneAxisServoVel : public OneAxisServo {
  public:
    /** min and max values are understood as travel bounds. 
	The zero position is (max-min)/2
        @param power is the maximal torque the servo can generate
        @param maxVel is understood as a speed parameter of the servo.
        @param damping adjusts the power of the servo in dependence of the distance
         to the set point. This regulates the damping and the body feeling
          0: the servo has no power at the set point (maximal body feeling);
          1: is servo has full power at the set point: perfectly damped.

    */
    OneAxisServoVel(const OdeHandle& odeHandle, 
		    OneAxisJoint* joint, double _min, double _max, 
		    double power, double damp=0.05, double maxVel=20, double jointLimit = 1.3)
      : OneAxisServo(joint, _min, _max, maxVel/2, 0, 0, 0, jointLimit, false),
        // don't wonder! It is correct to give maxVel as a power parameter.
        motor(odeHandle, joint, power), power(power), damp(clip(damp,0.0,1.0))
    {            
    }

    virtual ~OneAxisServoVel(){}

    /** adjusts the power of the servo*/
    virtual void setPower(double _power) { 
      power=_power;
      motor.setPower(power);
    };
    /** returns the power of the servo*/
    virtual double getPower() {             
      return power;
    };
    virtual double getDamping() { 
      return damp;
    };
    virtual void setDamping(double _damp) { 
      damp = clip(_damp,0.0,1.0);
    };
    /** offetCanceling does not exist for this type of servo */
    virtual double& offsetCanceling() { 
      dummy=0;
      return dummy;
    };

    /** adjusts maximal speed of servo*/
    virtual void setMaxVel(double maxVel) { 
      this->maxVel = maxVel;
      pid.KP=maxVel/2;
    };
    /** adjusts maximal speed of servo*/
    virtual double getMaxVel() { 
      return maxVel;      
    };


    /** sets the set point of the servo. 
	Position must be between -1 and 1. It is scaled to fit into min, max, 
	however 0 is just in the center of min and max
    */
    virtual void set(double pos){
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
    }
    
    /** returns the position of the servo in ranges [-1, 1] (scaled by min, max, centered)*/
    virtual double get(){
      double pos =  joint->getPosition1();      
      return 2*(pos-min)/(max-min) - 1;    
    }    
  protected:
    AngularMotor1Axis motor;         
    double dummy;
    double power;
    double damp;
  };
    
}
#endif
