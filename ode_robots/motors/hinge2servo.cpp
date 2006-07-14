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
 *   Revision 1.1.2.2  2006/01/03 10:35:52  fhesse
 *   getAngle1() -> getPosition1(); the same with getAngle1Rate()
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
#include "hinge2servo.h"
#include <assert.h>

namespace lpzrobots {

Hinge2Servo::Hinge2Servo(Hinge2Joint* joint, double min, double max, double power)
  : pid(power, 2.0, 0.3 ), joint(joint)
{
  assert(min <= 0 && min <= max);
  this->min = min;
  this->max = max;
}

void Hinge2Servo::set(double pos){
  if(pos > 0){
    pos *= max; 
  }else{
    pos *= -min;
  }
  pid.setTargetPosition(pos);  
  double force = pid.stepWithD(joint->getPosition1(), joint->getPosition1Rate());
  joint->addTorques(force, 0);
}

double Hinge2Servo::get(){
  double pos = joint->getPosition1(); 
  if(pos > 0){
    pos /= max; 
  }else{
    pos /= -min;
  }
  return pos;
}

}  
