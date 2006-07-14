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
 *   Revision 1.2  2006-07-14 12:23:32  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.3  2006/02/07 15:51:56  martius
 *   axis, setpower
 *
 *   Revision 1.1.2.2  2006/01/03 10:42:17  fhesse
 *   get...1() -> getPosition1()
 *
 *   Revision 1.1.2.1  2005/12/20 17:53:42  martius
 *   changed to Joints from joint.h
 *   new servos for universal and hinge2
 *
 *   Revision 1.1  2005/09/12 00:08:45  martius
 *   servo for hinges
 *
 *                                                                 *
 ***************************************************************************/
#include "universalservo.h"
#include <assert.h>

namespace lpzrobots {

  UniversalServo::UniversalServo(UniversalJoint* joint, double min1, double max1, double power1,
				 double min2, double max2, double power2)
    : pid1(power1, 2.0, 0.3 ), pid2(power2, 2.0, 0.3 ), joint(joint)
  {
    assert(min1 <= 0 && min1 <= max1);
    assert(min2 <= 0 && min2 <= max2);
    this->min1 = min1;
    this->max1 = max1;
    this->min2 = min2;
    this->max2 = max2;
  }

  void UniversalServo::set(double pos1, double pos2){
    if(pos1 > 0){
      pos1 *= max1; 
    }else{
      pos1 *= -min1;
    }
    pid1.setTargetPosition(pos1);  
    double force1 = pid1.stepWithD(joint->getPosition1(), joint->getPosition1Rate());
    if(pos2 > 0){
      pos2 *= max2; 
    }else{
      pos2 *= -min2;
    }
    pid2.setTargetPosition(pos2);  
    double force2 = pid2.stepWithD(joint->getPosition2(), joint->getPosition2Rate());

    joint->addTorques(force1, force2);
  }

  double UniversalServo::get1(){
    double pos = joint->getPosition1(); 
    if(pos > 0){
      pos /= max1; 
    }else{
      pos /= -min1;
    }
    return pos;
  }

  double UniversalServo::get2(){
    double pos = joint->getPosition2(); 
    if(pos > 0){
      pos /= max2; 
    }else{
      pos /= -min2;
    }
    return pos;
  }

  void UniversalServo::get(double& p1, double& p2){
    p1 = get1();
    p2 = get2();
  }

  /* adjusts the power of the two servos*/
  void UniversalServo::setPower(double power1, double power2){
    pid1.setKP(power1);
    pid2.setKP(power2);    
  }


}  
