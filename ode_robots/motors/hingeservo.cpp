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
 *   Revision 1.1.4.4  2006-02-07 15:51:56  martius
 *   axis, setpower
 *
 *   Revision 1.1.4.3  2006/01/10 14:47:57  martius
 *   *** empty log message ***
 *
 *   Revision 1.1.4.2  2006/01/02 08:24:12  fhesse
 *   getAngle() changed to getPosition1();
 *   the same with angle rate
 *
 *   Revision 1.1.4.1  2005/12/20 17:53:42  martius
 *   changed to Joints from joint.h
 *   new servos for universal and hinge2
 *
 *   Revision 1.1  2005/09/12 00:08:45  martius
 *   servo for hinges
 *
 *                                                                 *
 ***************************************************************************/
#include "hingeservo.h"
#include <assert.h>

namespace lpzrobots {

  HingeServo::HingeServo(HingeJoint* joint, double min, double max, double power)
    : pid(power, 2.0, 0.3 ), joint(joint)
  {
    assert(min <= 0 && min <= max);
    this->min = min;
    this->max = max;
  }

  void HingeServo::set(double pos){ 
    if(pos > 0){ 
      pos *= max; 
    }else{
      pos *= -min;
    }
    pid.setTargetPosition(pos);  
    double force = pid.stepWithD(joint->getPosition1(), joint->getPosition1Rate());
    joint->addTorque(force);
  }

  double HingeServo::get(){
    double pos = joint->getPosition1();    
    if(pos > 0){
      pos /= max; 
    }else{
      pos /= -min;
    }
    return pos;
  }

  void HingeServo::setPower(double power){
    pid.setKP(power);
  }

  
}
