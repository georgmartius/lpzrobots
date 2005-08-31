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
 *   Revision 1.3  2005-08-31 11:14:06  martius
 *   removed unused vars
 *
 *   Revision 1.2  2005/08/03 20:38:56  martius
 *   added textures and correct placement
 *
 *   Revision 1.1  2005/07/29 15:13:11  martius
 *   a robot with 4 independent wheels
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __NIMM4_H
#define __NIMM4_H

#include "abstractrobot.h"

/** Robot that looks like a Nimm 2 Bonbon :-)
    2 wheels and a cylinder like body   
*/
class Nimm4 : public AbstractRobot{
public:
  
  Nimm4(dWorldID w, dSpaceID s, dJointGroupID c, double size=1, double force=3, double speed=15, bool sphereWheels=true);

  virtual ~Nimm4(){};

  /**
   * draws the vehicle
   */
  virtual void draw();

  /** sets the vehicle to position pos, sets color to c, and creates robot if necessary
      @params pos desired position of the robot in struct Position
      @param c desired color for the robot in struct Color
  */
  virtual void place(Position pos , Color *c = 0);

  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1] 
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  virtual int getSensors(sensor* sensors, int sensornumber);

  /** sets actual motorcommands
      @param motors motors scaled to [-1,1] 
      @param motornumber length of the motor array
  */
  virtual void setMotors(const motor* motors, int motornumber);

  /** returns number of sensors
   */
  virtual int getSensorNumber(){
    return sensorno;
  };

  /** returns number of motors
   */
  virtual int getMotorNumber(){
    return motorno;
  };

  /** returns position of robot 
      @return position robot position in struct Position  
  */
  virtual Position getPosition();

  /** returns a vector with the positions of all segments of the robot
      @param poslist vector of positions (of all robot segments) 
      @return length of the list
  */
  virtual int getSegmentsPosition(vector<Position> &poslist);

  virtual bool collisionCallback(void *data, dGeomID o1, dGeomID o2);

  /** sets the textures used for body and wheels
  */
  virtual void setTextures(int body, int wheels);

protected:

  /** creates vehicle at desired position 
      @param pos struct Position with desired position
  */
  virtual void create(Position pos); 

  /** destroys vehicle and space
   */
  virtual void destroy();
  static void mycallback(void *data, dGeomID o1, dGeomID o2);

  double length;  // chassis length
  double width;  // chassis width
  double height;   // chassis height
  double radius;  // wheel radius
  double wheelthickness; // thickness of the wheels  
  bool sphereWheels; // draw spherical wheels?
  double cmass;    // chassis mass
  double wmass;    // wheel mass
  int sensorno;      //number of sensors
  int motorno;       // number of motors
  int segmentsno;    // number of motorsvehicle segments
  double speed;    // 

  double max_force;        // maximal force for motors

  int bodyTexture;
  int wheelTexture;

  bool created;      // true if robot was created

  Object object[5];  // 1 cylinder, 4 wheels
  dJointID joint[4]; // joints between cylinder and each wheel

  dSpaceID car_space;
};

#endif
