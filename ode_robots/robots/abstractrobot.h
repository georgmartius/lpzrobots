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
 *   Revision 1.8  2005-09-12 00:10:44  martius
 *   position operators are const
 *
 *   Revision 1.7  2005/08/30 16:53:53  martius
 *   Position struct has toArray and operators
 *
 *   Revision 1.6  2005/08/29 06:40:35  martius
 *   added virtual destructor
 *
 *   Revision 1.5  2005/08/22 20:32:45  martius
 *   robot has a name
 *
 *   Revision 1.4  2005/07/27 13:22:16  martius
 *   position and color have constructors
 *   ODEHandle
 *
 *   Revision 1.3  2005/07/18 14:47:41  martius
 *   world, space, contactgroup are not pointers anymore.
 *
 *   Revision 1.2  2005/07/07 09:27:11  martius
 *   isGeomInObjectList added
 *
 *   Revision 1.1  2005/06/15 14:20:04  martius
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

typedef struct Color
{
  Color() {r=g=b=0;};
  Color(double _r, double _g, double _b){ r=_r; g=_g; b=_b; }
  double r;
  double g;
  double b;
} Color;

typedef struct Position
{
  Position(){x=y=z=0;}
  Position(double _x, double _y, double _z){ x=_x; y=_y; z=_z; }
  ///  p MUST have a size of at least 3 
  Position(const double* p){ x=p[0]; y=p[1]; z=p[2]; } 
  const double* toArray(){ array[0]=x;array[1]=y; array[2]=z; return array; } 
  Position operator+(const Position& sum) const 
           { Position rv(x+sum.x, y+sum.y, z+sum.z); return rv; }
  Position operator-(const Position& sum) const
           { Position rv(x-sum.x, y-sum.y, z-sum.z); return rv; }
  Position operator*(double f) const { Position rv(x*f, y*f, z*f); return rv; }

  double x;
  double y;
  double z;
  double array[3];
} Position;

typedef struct
{
  dBodyID body;
  dGeomID geom;
} Object;

/** Data structure for accessing the ODE */
typedef struct ODEHandle
{
  ODEHandle(  dWorldID _world, dSpaceID _space, dJointGroupID _jointGroup){
    world = _world; space = _space; jointGroup= _jointGroup;
  }
  dWorldID world;
  dSpaceID space;
  dJointGroupID jointGroup;
} ODEHandle;

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
  AbstractRobot(dWorldID w, dSpaceID s, dJointGroupID c, const char* name="abstractRobot")
    : name(name) {
    world=w;
    space=s;
    contactgroup=c;
  };

  AbstractRobot(const ODEHandle& odehandle, const char* name="abstractRobot")
    : name(name) {
    world=odehandle.world;
    space=odehandle.space;
    contactgroup=odehandle.jointGroup;
  };

  virtual ~AbstractRobot(){}


  /// returns the name of the robot
  const char* getName() const { return name;}

  /// draws the robot
  virtual void draw() = 0;

/** sets the vehicle to position pos, sets color to c, and creates robot if necessary
    @params pos desired position of the robot in struct Position
    @param c desired color for the robot in struct Color. Might be NULL!
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
  /// returns the name of the robot
  void setName(const char* name) { this->name = name; }

 protected:
  static bool isGeomInObjectList(Object* os, int len, dGeomID geom){  
    for(int i=0; i < len; i++){
      if(geom == os[i].geom) return true;
    }
    return false;
  }

 protected:

  dSpaceID space;
  dWorldID world;
  dJointGroupID contactgroup;
  const char* name;
  

  Color color;

} ;



 #endif
 
