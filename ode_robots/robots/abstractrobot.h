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
 *   Revision 1.1  2005-06-15 14:20:04  martius
 *   moved into robots
 *                                                                 *
 ***************************************************************************/
#ifndef __ABSTRACTROBOT_H
#define __ABSTRACTROBOT_H

#include <ode/common.h>

#include <vector>
using namespace std;
 
typedef double sensor;
typedef double motor;

typedef struct
{
	double r;
	double g;
	double b;
} Color;

typedef struct
{
	double x;
	double y;
	double z;
} Position;

typedef struct
{
  dBodyID body;
  dGeomID geom;
} Object;


/**
 * Abstract class (interface) for robot 
 * 
 * 
 */
class AbstractRobot{
public:


  /**
   * Constructor
   * @param w world in which robot should be created
   * @param s space in which robot should be created
   * @param c contactgroup for collision treatment
   */
  AbstractRobot(dWorldID *w, dSpaceID *s, dJointGroupID *c){
    world=w;
    space=s;
    contactgroup=c;
  };

  /// draws the robot
  virtual void draw() = 0;

/** sets the vehicle to position pos, sets color to c, and creates robot if necessary
    @params pos desired position of the robot in struct Position
    @param c desired color for the robot in struct Color
*/
  virtual void place(Position pos , Color *c = 0) = 0;

  /** checks for internal collisions and treats them. 
   *  In case of a treatment return true (collision will be ignored by other objects and the default routine)
   *  else false (collision is passed to other objects and (if not treated) to the default routine).
   */
  virtual bool collisionCallback(void *data, dGeomID o1, dGeomID o2){
    return false;
  }
  

  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1] 
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  virtual int getSensors(sensor* sensors, int sensornumber)=0;

  /** sets actual motorcommands
      @param motors motors scaled to [-1,1] 
      @param motornumber length of the motor array
  */
  virtual void setMotors(const motor* motors, int motornumber)=0;

  /** returns number of sensors
  */
  virtual int getSensorNumber()=0;

  /** returns number of motors
  */
  virtual int getMotorNumber()=0;

  /** returns position of robot 
      @param pos vector of desired position (x,y,z)
   */
  virtual Position getPosition()=0;

  /** returns a vector with the positions of all segments of the robot
      @param vector of positions (of all robot segments) 
      @return length of the list
  */
  virtual int getSegmentsPosition(vector<Position> &poslist) = 0;

  /** sets color of the robot
      @param col Color struct with desired Color
   */
  virtual void setColor(Color col){
    color=col;
  };

 protected:

  dSpaceID *space;
  dWorldID *world;

  dJointGroupID *contactgroup;

  Color color;

} ;



 #endif
 
