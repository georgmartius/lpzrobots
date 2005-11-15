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
 *   Revision 1.6.4.2  2005-11-15 12:29:26  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.6.4.1  2005/11/14 17:37:17  martius
 *   moved to selforg
 *
 *   Revision 1.6  2005/11/09 13:26:21  martius
 *   added factorSensor
 *
 *   Revision 1.5  2005/10/06 17:14:24  martius
 *   switched to stl lists
 *
 *   Revision 1.4  2005/09/22 12:24:37  martius
 *   removed global variables
 *   OdeHandle and GlobalData are used instead
 *   sensor prepared
 *
 *   Revision 1.3  2005/08/31 17:18:15  fhesse
 *   setTextures added, Mass is now sphere (not box anymore)
 *
 *   Revision 1.2  2005/07/26 17:04:21  martius
 *   lives in its own space now
 *
 *   Revision 1.1  2005/07/21 12:17:04  fhesse
 *   new hurling snake, todo: add collision space, clean up, comment
 *
 *         
 *                                                                 *
 ***************************************************************************/
#ifndef __HURLINGSNAKE_H
#define __HURLINGSNAKE_H

#include "oderobot.h"
#include <selforg/configurable.h>

/**
 * Hurling snake is a string a beats.
 * 
 */
class HurlingSnake : public OdeRobot, public Configurable{
public:


  /**
   * Constructor
   * @param w world in which robot should be created
   * @param s space in which robot should be created
   * @param c contactgroup for collision treatment
   */
  HurlingSnake(const OdeHandle& odeHandle);

  /** sets the textures used for body and wheels
   */
  virtual void setTextures(int body);


  /// draws the robot
  virtual void draw();

/** sets the vehicle to position pos, sets color to c, and creates robot if necessary
    @params pos desired position of the robot in struct Position
    @param c desired color for the robot in struct Color
*/
  virtual void place(Position pos , Color *c = 0);

  /** checks for internal collisions and treats them. 
   *  In case of a treatment return true (collision will be ignored by other objects and the default routine)
   *  else false (collision is passed to other objects and (if not treated) to the default routine).
   */
  virtual bool collisionCallback(void *data, dGeomID o1, dGeomID o2);
  /** this function is called in each timestep. It should perform robot-internal checks, 
      like space-internal collision detection, sensor resets/update etc.
      @param GlobalData structure that contains global data from the simulation environment
   */
  virtual void doInternalStuff(const GlobalData& globalData);
  

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
  virtual int getSensorNumber();

  /** returns number of motors
  */
  virtual int getMotorNumber();

  /** returns a vector with the positions of all segments of the robot
      @param vector of positions (of all robot segments) 
      @return length of the list
  */
  virtual int getSegmentsPosition(vector<Position> &poslist);
  
  /// returns the name of the object (with version number)
  virtual paramkey getName() const {return name; } 
  
  /** The list of all parameters with there value as allocated lists.
      @param keylist,vallist will be allocated with malloc (free it after use!)
      @return length of the lists
  */
  virtual paramlist getParamList() const;
  
  virtual paramval getParam(const paramkey& key) const;

  virtual bool setParam(const paramkey& key, paramval val);


 private:

  virtual Object getMainObject(){ return object[4]; }

  /** creates robot at desired position 
      @param pos struct Position with desired position
  */
  virtual void create(Position pos); 

  /** destroys robot and space
   */
  virtual void destroy();

  static void mycallback(void *data, dGeomID o1, dGeomID o2);
       
  bool created;      // true if robot was created


  Position initial_pos;    // initial position of robot

  int NUM;	   /* number of boxes */
  double SIDE;     /* side length of a box */
  double MASS;	   /* mass of a box */
  double RADIUS;   /* sphere radius */

  dJointID joint[9];
  Object object[10];

  double old_position[3];

  int sensorno;
  int motorno;

  paramval factorForce;
  paramval frictionGround;
  paramval factorSensor;

  dSpaceID snake_space;

  int bodyTexture;

} ;



#endif
 
