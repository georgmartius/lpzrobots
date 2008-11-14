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
 *   Revision 1.5  2008-11-14 11:23:05  martius
 *   added centered Servos! This is useful for highly nonequal min max values
 *   skeleton has now also a joint in the back
 *
 *   Revision 1.4  2007/07/17 07:17:40  martius
 *   joints limits are set
 *   damping is accessable for both axis
 *
 *   Revision 1.3  2007/07/03 13:02:22  martius
 *   maximum velocity check
 *   new pid with stepsize
 *
 *   Revision 1.2  2007/04/03 16:31:02  der
 *   use fixed version of pid
 *   new default values
 *
 *   Revision 1.1  2007/02/12 13:28:20  martius
 *   twoaxisservo and some minor changes
 *
 *
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __SERVO2_H
#define __SERVO2_H

#include "joint.h"
#include "pid.h"

namespace lpzrobots {

  /** general servo motor for 2 axis joints
   */
  class TwoAxisServo {
  public:
    /** min and max values are understood as travel bounds. Min should be less than 0.*/
  
    TwoAxisServo(TwoAxisJoint* joint, double _min1, double _max1, double power1, 
		 double _min2, double _max2, double power2, 
		 double damp=0.2, double integration=2, double maxVel=10.0, 
		 bool minmaxCheck=true)
      : joint(joint),
	pid1(power1, integration, damp),
	pid2(power2, integration, damp),  
	maxVel(maxVel) { 
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
      joint->getPart1()->limitLinearVel(maxVel);
      joint->getPart2()->limitLinearVel(maxVel);
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
    virtual double& power1() { 
      return pid1.KP;
    };
    /** returns the power of the servo*/
    virtual double& power2() { 
      return pid2.KP;
    };

    /** returns the damping of the servo*/
    virtual double& damping1() { 
      return pid1.KD;
    };

    /** returns the damping of the servo*/
    virtual double& damping2() { 
      return pid2.KD;
    };

    /** returns the damping of the servo*/
    virtual double& offsetCanceling() { 
      return pid1.KI; 
    };

    virtual void setMinMax1(double _min, double _max){
      min1=_min;
      max1=_max;
      joint->setParam(dParamLoStop, _min * 1.3);
      joint->setParam(dParamHiStop, _max * 1.3);
    }

    virtual void setMinMax2(double _min, double _max){
      min2=_min;
      max2=_max;
      joint->setParam(dParamLoStop2, _min * 1.3);
      joint->setParam(dParamHiStop2, _max * 1.3);
    }

  
  protected:
    TwoAxisJoint* joint;
    double min1;
    double max1;
    double min2;
    double max2;
    PID pid1;
    PID pid2;
    double maxVel;
  };

  typedef TwoAxisServo UniversalServo;


  /** general servo motor for 2 axis joints with zero position centered
   */
  class TwoAxisServoCentered : public TwoAxisServo {
  public:
    /** min and max values are understood as travel bounds. 
	The zero position is max-min/2
    */
    TwoAxisServoCentered(TwoAxisJoint* joint, double _min1, double _max1, double power1, 
			 double _min2, double _max2, double power2, 
			 double damp=0.2, double integration=2, double maxVel=10.0)
      : TwoAxisServo(joint, _min1, _max1, power1, _min2, _max2, power2,
		     damp, integration, maxVel, false){      
    }
    virtual ~TwoAxisServoCentered(){}

    /** sets the set point of the servo. 
	Position must be between -1 and 1. It is scaled to fit into min, max, 
	however 0 is just in the center of min and max
    */
    virtual void set(double pos1, double pos2){
      pos1 = (pos1+1)*(max1-min1)/2 + min1;

      pid1.setTargetPosition(pos1);  
      // double force1 = pid1.stepWithD(joint->getPosition1(), joint->getPosition1Rate());
      double force1 = pid1.step(joint->getPosition1(), joint->odeHandle.getTime());
      // limit force to 1*KP
      force1 = std::min(pid1.KP, std::max(-pid1.KP,force1));// limit force to 1*KP

      pos2 = (pos2+1)*(max2-min2)/2 + min2;
      pid2.setTargetPosition(pos2);  
      //      double force2 = pid2.stepWithD(joint->getPosition2(), joint->getPosition2Rate());
      double force2 = pid2.step(joint->getPosition2(), joint->odeHandle.getTime());
      // limit force to 1*KP
      force2 = std::min(pid2.KP, std::max(-pid2.KP,force2));// limit force to 1*KP
      joint->addForces(force1, force2);
      joint->getPart1()->limitLinearVel(maxVel);
      joint->getPart2()->limitLinearVel(maxVel);
    }

    /** returns the position of the servo (joint) of 1. axis in ranges [-1, 1] 
	(scaled by min1, max1, centered)*/
    virtual double get1(){
      double pos =  joint->getPosition1();      
      return 2*(pos-min1)/(max1-min1) + 1;    
    }


    /** returns the position of the servo (joint) of 2. axis in ranges [-1, 1]
	(scaled by min2, max2, centered)*/
    virtual double get2(){
      double pos =  joint->getPosition2();
      return 2*(pos-min2)/(max2-min2) + 1;    
    }
    
  };



}
#endif
