/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Rald Der       <ralfder at mis dot mpg dot de>                       *
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
 ***************************************************************************/
#ifndef __ODEROBOT_H
#define __ODEROBOT_H

#include <vector>
 
#include <selforg/abstractrobot.h>
#include <selforg/storeable.h>
#include "odehandle.h"
#include "osghandle.h"
#include "globaldata.h"
#include "color.h"
#include "pos.h"
#include "osgforwarddecl.h"

namespace lpzrobots {
  
  class Primitive;
  class Joint;

  /**
   * Abstract class  for ODE robots
   * 
   */
  class OdeRobot : public AbstractRobot, public Storeable {
  public:

    friend class OdeAgent;
//    friend class AtomOdeAgent;

    /**
     * Constructor
     */
    OdeRobot(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
	     const std::string& name, const std::string& revision);

    /// calls cleanup()
    virtual ~OdeRobot();


    /// update the OSG notes here
    virtual void update() = 0;

    /** sets the vehicle to position pos
	@param pos desired position of the robot
    */
    virtual void place(const Pos& pos);

    /** sets the pose of the vehicle
	@param pose desired 4x4 pose matrix
    */
    virtual void place(const osg::Matrix& pose) = 0;

    /** @deprecated
     *  Do not use it anymore, collision control is done automatically. 
     *  In case of a treatment return true 
     *  (collision will be ignored by other objects and the default routine)
     *  else false (collision is passed to other objects and (if not treated) to the default routine).
     */
    virtual bool collisionCallback(void *data, dGeomID o1, dGeomID o2){ return false; };

    /** this function is called each controlstep before control. 
	This is the place the perform active sensing (e.g. Image processing)
	@param globalData structure that contains global data from the simulation environment
    */
    virtual void sense(GlobalData& globalData) {};

    /** this function is called in each simulation timestep (always after control). It
	should perform robot-internal checks and actions
	like resetting certain sensors or implement velocity dependend friction and the like.
	The attached Motors should act here.
	@param globalData structure that contains global data from the simulation environment
    */
    virtual void doInternalStuff(GlobalData& globalData) {};

    /** sets color of the robot
	@param col Color struct with desired Color
    */
    virtual void setColor(const Color& col);

    
    /*********** BEGIN TRACKABLE INTERFACE ****************/
    
    /** returns position of the object
	@return vector of position (x,y,z)
    */
    virtual Position getPosition() const;
    
    /** returns linear speed vector of the object
	@return vector  (vx,vy,vz)
    */
    virtual Position getSpeed() const;
    
    /** returns angular velocity vector of the object
	@return vector  (wx,wy,wz)
    */
    virtual Position getAngularSpeed() const;
    
    /** returns the orientation of the object
	@return 3x3 rotation matrix
    */
    virtual matrix::Matrix getOrientation() const;
    /*********** END TRACKABLE INTERFACE ****************/
    
    /// return the primitive of the robot that is used for tracking and camera following
    virtual Primitive* getMainPrimitive() const  { 
      if (!objects.empty()) return objects[0]; else return 0;
    };

    /// returns a list of all primitives of the robot (used to store and restore the robot)
    virtual std::vector<Primitive*> getAllPrimitives() const { return objects; };
    /// returns a list of all primitives of the robot (const version) (used to store and restore the robot)

    /* ********** STORABLE INTERFACE **************** */
    virtual bool store(FILE* f) const;
    
    virtual bool restore(FILE* f);  
    /* ********** END STORABLE INTERFACE ************ */

    /** relocates robot to new postion (keep current pose)
        such that lowest body part is at the given position 
        (the center of it, so the bounding box is not checked)
     */
    virtual void moveToPosition(Pos pos = Pos(0,0,0.5));

  protected:
    
    static bool isGeomInPrimitiveList(Primitive** ps, int len, dGeomID geom);
    static bool isGeomInPrimitiveList(std::list<Primitive*> ps, dGeomID geom);

    /// deletes all objects (primitives) and joints (is called automatically in destructor)
    virtual void cleanup();

  protected:
    /// list of objects (should be populated by subclasses)
    std::vector <Primitive*> objects;
    /// list of joints (should be populated by subclasses)
    std::vector <Joint*> joints;


    OdeHandle odeHandle;
    OsgHandle osgHandle;
    dSpaceID parentspace;
  };

}

#endif
 
