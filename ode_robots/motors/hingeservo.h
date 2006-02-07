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
 *   Revision 1.1.4.2  2006-02-07 15:51:56  martius
 *   axis, setpower
 *
 *   Revision 1.1.4.1  2005/12/20 17:53:42  martius
 *   changed to Joints from joint.h
 *   new servos for universal and hinge2
 *
 *   Revision 1.1  2005/09/12 00:08:28  martius
 *   servo for hinges
 *                                                                 *
 ***************************************************************************/
#ifndef __HINGESERVO_H
#define __HINGESERVO_H

#include "pid.h"
#include "joint.h"

namespace lpzrobots {

/** PID Servo motor for hinge joints.
*/
class HingeServo {
public:
  /** min and max values are understood as travel bounds.
      power is the power of the servo
      */
  HingeServo(HingeJoint* joint, double min, double max, double power);

  /** sets the set point of the servo. 
      Position must be between -1 and 1. It is scaled to fit into min, max
  */
  void set(double position);
  /** returns the position of the slider in ranges [-1, 1] (scaled by min, max)*/
  double get();
  
  /* adjusts the power of the servo*/
  void setPower(double power);
  
private:
  PID pid;
  double min;
  double max;
  HingeJoint* joint;
};

}
#endif
