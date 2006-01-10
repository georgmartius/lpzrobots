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
 *                                                                 *
 ***************************************************************************
 *                                                                         *
 * Spherical Robot magically driven                                        *
 *                                                                         *
 *   $Log$
 *   Revision 1.2.4.4  2006-01-10 22:25:09  martius
 *   moved to osg
 *
 *
 *                                                                 *
 ***************************************************************************/

#ifndef __FORCESSPHERE_H
#define __FORCESSPHERE_H

#include "oderobot.h"

namespace lpzrobots {

  class Primitive;

  class ForcedSphere : public OdeRobot
  {
  protected:
    Primitive* object[1];
    double radius;
    double max_force;
    bool created;

  public:
  
    /**
     *constructor
     **/ 
    ForcedSphere ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
		   const char* name, double radius=1, double max_force=1);
  
    virtual ~ForcedSphere();
	
    /// update all primitives and joints
    virtual void update();

    /** sets the pose of the vehicle
	@params pose desired 4x4 pose matrix
    */
    virtual void place(const osg::Matrix& pose);
  
    /**
     *This is the collision handling function for snake robots.
     *This overwrides the function collisionCallback of the class robot.
     *@param data
     *@param o1 first geometrical object, which has taken part in the collision
     *@param o2 second geometrical object, which has taken part in the collision
     *@return true if the collision was threated  by the robot, false if not
     **/
    virtual bool collisionCallback(void *data, dGeomID o1, dGeomID o2);
    /** this function is called in each timestep. It should perform robot-internal checks, 
	like space-internal collision detection, sensor resets/update etc.
	@param GlobalData structure that contains global data from the simulation environment
    */
    virtual void doInternalStuff(const GlobalData& globalData);
	
    /**
     *Writes the sensor values to an array in the memory.
     *@param sensor* pointer to the array
     *@param sensornumber length of the sensor array
     *@return number of actually written sensors
     **/
    virtual int getSensors ( sensor* sensors, int sensornumber );
	
    /**
     *Reads the actual motor commands from an array, an sets all motors of the snake to this values.
     *It is an linear allocation.
     *@param motors pointer to the array, motor values are scaled to [-1,1] 
     *@param motornumber length of the motor array
     **/
    virtual void setMotors ( const motor* motors, int motornumber );
	
    /**
     *Returns the number of motors used by the snake.
     *@return number of motors
     **/
    virtual int getMotorNumber();
  
    /**
     *Returns the number of sensors used by the robot.
     *@return number of sensors
     **/
    virtual int getSensorNumber();
	
 
  protected:
    /** the main object of the robot, which is used for position and speed tracking */
    virtual Primitive* getMainPrimitive() const { return object[0]; }

    /** creates vehicle at desired pose
	@param pose 4x4 pose matrix
    */
    virtual void create(const osg::Matrix& pose); 
    virtual void destroy(); 


  };

}

#endif
