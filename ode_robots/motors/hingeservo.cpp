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
 *   Revision 1.1  2005-09-12 00:08:45  martius
 *   servo for hinges
 *
 *                                                                 *
 ***************************************************************************/
#include "hingeservo.h"
#include <assert.h>

HingeServo::HingeServo(dJointID joint, double min, double max, double power)
  : PID(power, 2.0, 0.3 ) 
{
  assert(min <= 0);
  this->joint = joint;
  this->min = min;
  this->max = max;
}

void HingeServo::set(double pos){
  if(pos > 0){
    pos *= max; 
  }else{
    pos *= -min;
  }
  setTargetPosition(pos);  
  double force = stepWithD(dJointGetHingeAngle(joint), dJointGetHingeAngleRate(joint));
  dJointAddHingeTorque( joint, force);
}

double HingeServo::get(){
  double pos =  dJointGetHingeAngle(joint);    
  if(pos > 0){
    pos /= max; 
  }else{
    pos /= -min;
  }
  return pos;
}
  
