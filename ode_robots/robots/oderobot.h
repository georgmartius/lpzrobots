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
 *   Revision 1.8  2008-05-06 17:14:17  martius
 *   buildsystem further tuned,
 *   help in Makefile
 *   osg/data directory is also installed and registered at osg_robots
 *
 *   Revision 1.7  2007/12/06 10:02:49  der
 *   abstractground: returns now cornerpoints
 *   abstractobstacle: is now trackable
 *   hudstatistics: supports now AbstractmMeasure
 *
 *   Revision 1.6  2007/11/07 13:21:16  martius
 *   doInternal stuff changed signature
 *
 *   Revision 1.5  2007/04/05 15:11:42  martius
 *   angular speed tracking
 *
 *   Revision 1.4  2006/08/08 17:04:46  martius
 *   added new sensor model
 *
 *   Revision 1.3  2006/07/20 17:19:44  martius
 *   removed using namespace std from matrix.h
 *
 *   Revision 1.2  2006/07/14 12:23:41  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.17  2006/06/25 16:57:14  martius
 *   abstractrobot is configureable
 *   name and revision
 *
 *   Revision 1.1.2.16  2006/06/12 12:59:07  robot8
 *   -some corrections to component system (for example now connections are only pushed as pointers to the connection vector)
 *   -created the evolution simulation for evolutionary evolution algorithms with atom like structures
 *
 *   Revision 1.1.2.15  2006/03/30 12:34:56  martius
 *   documentation updated
 *
 *   Revision 1.1.2.14  2006/03/30 09:59:36  fhesse
 *   getOsgHandle() removed
 *   friend class OdeAgent added
 *
 *   Revision 1.1.2.13  2006/03/30 09:37:05  fhesse
 *   getOsgHandle added
 *
 *   Revision 1.1.2.12  2006/03/29 15:08:42  martius
 *   getMainPrimitive is public now
 *
 *   Revision 1.1.2.11  2005/12/30 22:54:16  martius
 *   collisioncallback must be implemented
 *
 *   Revision 1.1.2.10  2005/12/14 15:37:09  martius
 *   robots are working with osg
 *
 *   Revision 1.1.2.9  2005/12/13 18:11:40  martius
 *   still trying to port robots
 *
 *   Revision 1.1.2.8  2005/12/13 12:31:46  martius
 *   list version of isGeomInPrimitiveList
 *
 *   Revision 1.1.2.7  2005/12/11 23:35:08  martius
 *   *** empty log message ***
 *
 *   Revision 1.1.2.6  2005/12/06 10:13:25  martius
 *   openscenegraph integration started
 *
 *   Revision 1.1.2.5  2005/11/24 16:20:54  fhesse
 *   odeRto3x3RotationMatrix corrected
 *
 *   Revision 1.1.2.4  2005/11/16 11:26:52  martius
 *   moved to selforg
 *
 *   Revision 1.1.2.3  2005/11/15 12:29:27  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.1.2.2  2005/11/14 17:37:18  martius
 *   moved to selforg
 *
 *   Revision 1.1.2.1  2005/11/14 14:43:52  martius
 *   moved from abstractrobot to oderobot
 *
 *   Revision 1.11  2005/10/06 17:14:24  martius
 *   switched to stl lists
 *
 *   Revision 1.10  2005/09/22 12:24:36  martius
 *   removed global variables
 *   OdeHandle and GlobalData are used instead
 *   sensor prepared
 *
 *   Revision 1.9  2005/09/22 07:30:53  martius
 *   moved color and position into extra modules
 *
 *   Revision 1.8  2005/09/12 00:10:44  martius
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
#ifndef __ODEROBOT_H
#define __ODEROBOT_H

#include <vector>
 
#include <selforg/abstractrobot.h>
#include "odehandle.h"
#include "osghandle.h"
#include "globaldata.h"
#include "color.h"
#include "pos.h"
#include "osgforwarddecl.h"

namespace lpzrobots {
  
  class Primitive;

  /**
   * Abstract class  for ODE robots
   * 
   */
  class OdeRobot : public AbstractRobot {
  public:

    friend class OdeAgent;
//    friend class AtomOdeAgent;

    /**
     * Constructor
     */
    OdeRobot(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
	     const std::string& name, const std::string& revision);

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
    virtual bool collisionCallback(void *data, dGeomID o1, dGeomID o2) = 0;

    /** this function is called in each timestep after control. It
	should perform robot-internal checks and actions, 
	like acting and sensing of internal motors/sensors etc.
	@param globalData structure that contains global data from the simulation environment
    */
    virtual void doInternalStuff(GlobalData& globalData) = 0;

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
    virtual Primitive* getMainPrimitive() const  = 0;
    
  protected:
    
    static bool isGeomInPrimitiveList(Primitive** ps, int len, dGeomID geom);
    static bool isGeomInPrimitiveList(std::list<Primitive*> ps, dGeomID geom);


  protected:
    OdeHandle odeHandle;
    OsgHandle osgHandle;
    dSpaceID parentspace;
  };

}

#endif
 
