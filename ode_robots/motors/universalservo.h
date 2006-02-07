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
 *   Revision 1.1.2.2  2006-02-07 15:51:56  martius
 *   axis, setpower
 *
 *   Revision 1.1.2.1  2005/12/20 17:53:42  martius
 *   changed to Joints from joint.h
 *   new servos for universal and hinge2
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __UNIVERSALSERVO_H
#define __UNIVERSALSERVO_H

#include "pid.h"
#include "joint.h"

namespace lpzrobots {

/** PID Servo motor for universal joints.
*/
class UniversalServo {
public:
  /** min and max values are understood as travel bounds.
      power is the power of the servo
      */
  UniversalServo(UniversalJoint* joint, double min1, double max1, double power1,
	      double min2, double max2, double power2);

  /** sets the set point of the servo. 
      Position must be between -1 and 1. It is scaled to fit into min, max
  */
  void set(double position1, double position2);
  /** returns the position of the hinge in ranges [-1, 1] (scaled by min, max)*/
  double get1();
  /** returns the position of the hinge in ranges [-1, 1] (scaled by min, max)*/
  double get2();
  /** returns the position of the hinge in ranges [-1, 1] (scaled by min, max)*/
  void get(double& p1, double& p2);

  /* adjusts the power of the two servos*/
  void setPower(double power1, double power2);
  
private:
  PID pid1;
  PID pid2;
  double min1;
  double max1;
  double min2;
  double max2;
  UniversalJoint* joint;
};

}

#endif
