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
 *   Revision 1.6  2007-07-03 13:02:14  martius
 *   maximum velocity check
 *   new pid with stepsize
 *
 *   Revision 1.5  2007/04/03 16:29:24  der
 *   use fixed version of pid
 *   new default values
 *
 *   Revision 1.4  2007/02/23 15:14:17  martius
 *   *** empty log message ***
 *
 *   Revision 1.3  2007/02/21 16:07:23  der
 *   min and max are adjustable during runtime
 *   jointlimits are set by servo to 1.3 fold of servo limits
 *
 *   Revision 1.2  2007/02/12 13:28:20  martius
 *   twoaxisservo and some minor changes
 *
 *   Revision 1.1  2007/01/26 12:04:38  martius
 *   servos combinied into OneAxisServo
 *
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __SERVO1_H
#define __SERVO1_H

#include "joint.h"
#include "pid.h"

namespace lpzrobots {

  /** general servo motor
   */
  class OneAxisServo {
  public:
    /** min and max values are understood as travel bounds. Min should be less than 0.*/
  
    OneAxisServo(OneAxisJoint* joint, double _min, double _max, 
		 double power, double damp=0.2, double integration=2, double maxVel=10.0)
      : joint(joint), pid(power, integration, damp), maxVel(maxVel) { 
      assert(joint); 
      setMinMax(_min,_max);
      assert(min<max);
      assert(min <= 0);
      assert(max >= 0);
      assert(power>=0 && damp >=0 && integration >=0);
    }

    virtual ~OneAxisServo(){}

    /** sets the set point of the servo. 
	Position must be between -1 and 1. It is scaled to fit into min, max
    */
    virtual void set(double pos){
      if(pos > 0){
	pos *= max; 
      }else{
	pos *= -min;
      }
      pid.setTargetPosition(pos);  
      
      double force = pid.step(joint->getPosition1(), joint->odeHandle.getTime());
      force = std::min(pid.KP, std::max(-pid.KP,force));// limit force to 1*KP
      joint->addForce1(force);
      joint->getPart1()->limitLinearVel(maxVel);
      joint->getPart2()->limitLinearVel(maxVel);
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
      joint->setParam(dParamLoStop, min * 1.3);
      joint->setParam(dParamHiStop, max * 1.3);
    }

    /** adjusts the power of the servo*/
    virtual void setPower(double power) { 
      pid.KP = power;
    };
    /** returns the power of the servo*/
    virtual double& power() { 
      return pid.KP;
    };

    /** returns the damping of the servo*/
    virtual double& damping() { 
      return pid.KD;
    };
    /** returns the damping of the servo*/
    virtual double& offsetCanceling() { 
      return pid.KI;
    };
  
  private:
    OneAxisJoint* joint;
    double min;
    double max;
    PID pid;
    double maxVel;
  };

  typedef OneAxisServo SliderServo;
  typedef OneAxisServo HingeServo;
  typedef OneAxisServo Hinge2Servo;

}
#endif
