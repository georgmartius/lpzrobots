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
 *   Revision 1.1.2.1  2005/12/20 17:53:42  martius
 *   changed to Joints from joint.h
 *   new servos for universal and hinge2
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __HINGE2SERVO_H
#define __HINGE2SERVO_H

#include "pid.h"
#include "joint.h"

namespace lpzrobots {

/** PID Servo motor for hinge2 joints at axis 1 (steering axis). 
*/
class Hinge2Servo {
public:
  /** min and max values are understood as travel bounds.
      power is the power of the servo
      */
  Hinge2Servo(Hinge2Joint* joint, double min, double max, double power);

  /** sets the set point of the servo. 
      Position must be between -1 and 1. It is scaled to fit into min, max
  */
  void set(double position);
  /** returns the position of the hinge in ranges [-1, 1] (scaled by min, max)*/
  double get();
  
private:
  PID pid;
  double min;
  double max;
  Hinge2Joint* joint;
};

}

#endif
