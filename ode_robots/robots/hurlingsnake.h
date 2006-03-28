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
 *   Revision 1.6.4.6  2006-03-28 14:20:28  fhesse
 *   getTracePrimitive() added
 *
 *   Revision 1.6.4.5  2005/12/30 22:54:38  martius
 *   removed parentspace!
 *
 *   Revision 1.6.4.4  2005/12/21 17:35:09  martius
 *   moved to osg
 *
 *   Revision 1.6.4.3  2005/11/16 11:26:52  martius
 *   moved to selforg
 *
 *   Revision 1.6.4.2  2005/11/15 12:29:26  martius
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
#include "primitive.h"
#include "joint.h"

namespace lpzrobots {

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
    HurlingSnake(const OdeHandle& odeHandle, const OsgHandle& osgHandle);
  
    /// update the subcomponents
    virtual void update();

    /** sets the pose of the vehicle
	@params pose desired 4x4 pose matrix
    */
    virtual void place(const osg::Matrix& pose);

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

    /** the main object of the robot, which is used for position and speed tracking */
    virtual Primitive* getMainPrimitive() const { return object[4] /*(center)*/; }

    /** the main object of the robot, which is used for position and speed tracking */
    virtual Primitive* getTracePrimitive() const { return object[NUM-1] /*(head element)*/; }


  private:

    /** creates vehicle at desired pose
	@param pose 4x4 pose matrix
    */
    virtual void create(const osg::Matrix& pose); 

    /** destroys robot and space
     */
    virtual void destroy();

    static void mycallback(void *data, dGeomID o1, dGeomID o2);
       
    bool created;      // true if robot was created


    Position initial_pos;    // initial position of robot

    int NUM;	   /* number of beats */
    double MASS;	   /* mass of a beats */
    double RADIUS;   /* sphere radius */

    Joint* joint[9];
    Primitive* object[10];

    Pos oldp;

    int sensorno;
    int motorno;

    paramval factorForce;
    paramval frictionGround;
    paramval factorSensor;

  };

}

#endif
 
