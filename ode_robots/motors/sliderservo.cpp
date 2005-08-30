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
 *   Revision 1.1  2005-08-30 16:55:48  martius
 *   servo motor for sliders
 *
 *                                                                 *
 ***************************************************************************/
#include "sliderservo.h"

SliderServo::SliderServo(dJointID joint, double min, double max)
  : PID(500, 0, 20) // TODO make force adjustable
{
  this->joint = joint;
  this->min = min > 0 ? 0 : min;
  this->max = max;
}

void SliderServo::set(double position){
  if(position > 0){
    position *= max; 
  }else{
    position *= -min;
  }
  setTargetPosition(position);
  double force = step(get());
  dJointAddSliderForce( joint , force );  
}

double SliderServo::get(){
  double position =  dJointGetSliderPosition (joint);    
  if(position > 0){
    position /= max; 
  }else{
    position /= -min;
  }
  return position;
}
  
