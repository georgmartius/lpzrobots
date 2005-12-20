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
 *   Revision 1.5.4.1  2005-12-20 17:53:42  martius
 *   changed to Joints from joint.h
 *   new servos for universal and hinge2
 *
 *   Revision 1.5  2005/09/11 18:52:46  martius
 *   new constants
 *
 *   Revision 1.4  2005/09/02 17:23:52  martius
 *   parameters adjusted
 *
 *   Revision 1.3  2005/09/02 13:20:14  martius
 *   adjusted parameters and usage of derivative of slider from ode
 *
 *   Revision 1.2  2005/09/01 14:22:00  martius
 *   parameters adjusted
 *
 *   Revision 1.1  2005/08/30 16:55:48  martius
 *   servo motor for sliders
 *
 *                                                                 *
 ***************************************************************************/
#include "sliderservo.h"
#include <assert.h>

namespace lpzrobots {

SliderServo::SliderServo(SliderJoint* joint, double min, double max, double mass)
  : pid(mass * 200.0, 2.0, 0.3 ), joint(joint)
{
  assert(min <= 0);
  this->joint = joint;
  this->min = min;
  this->max = max;
}

void SliderServo::set(double pos){
  if(pos > 0){
    pos *= max; 
  }else{
    pos *= -min;
  }
  pid.setTargetPosition(pos);  
  double force = pid.stepWithD(joint->getLength(), joint->getLengthRate());
  joint->addForce(force);  
}

double SliderServo::get(){
  double pos =  joint->getLength();
  if(pos > 0){
    pos /= max; 
  }else{
    pos /= -min;
  }
  return pos;
}
  
}
