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
 *   Revision 1.7  2009-02-02 16:07:49  martius
 *   added dummy object to avoid crash with tv/following camera
 *
 *   Revision 1.6  2007/11/07 13:21:16  martius
 *   doInternal stuff changed signature
 *
 *   Revision 1.5  2006/12/11 18:24:37  martius
 *   memory freed
 *
 *   Revision 1.4  2006/07/14 12:23:42  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.3.4.6  2006/03/30 12:34:57  martius
 *   documentation updated
 *
 *   Revision 1.3.4.5  2006/01/10 21:46:34  martius
 *   collcallbak
 *
 *   Revision 1.3.4.4  2006/01/10 20:32:58  martius
 *   moved to osg
 *
 *   Revision 1.3.4.3  2005/11/16 11:26:53  martius
 *   moved to selforg
 *
 *   Revision 1.3.4.2  2005/11/15 12:29:27  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.3.4.1  2005/11/14 17:37:18  martius
 *   moved to selforg
 *
 *   Revision 1.3  2005/09/22 12:24:37  martius
 *   removed global variables
 *   OdeHandle and GlobalData are used instead
 *   sensor prepared
 *
 *   Revision 1.2  2005/07/18 14:47:41  martius
 *   world, space, contactgroup are not pointers anymore.
 *
 *   Revision 1.1  2005/07/06 16:03:37  martius
 *   dummy robot that connects motors with sensors
 *
 ***************************************************************************/
#ifndef __SHORTCIRCUIT_H
#define __SHORTCIRCUIT_H


#include "oderobot.h"
#include "primitive.h"

namespace lpzrobots {

  /**
   * 
   */
  class ShortCircuit : public OdeRobot{
  public:
    ShortCircuit(const OdeHandle& odeHandle, const OsgHandle& osgHandle, int sensornumber, int motornumber);

    virtual ~ShortCircuit();

    virtual void update() {}

    /** sets the pose of the vehicle
	@param pose desired 4x4 pose matrix
    */
    virtual void place(const osg::Matrix& pose) {}

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
    virtual int getSensorNumber() {return sensorno; }

    /** returns number of motors
     */
    virtual int getMotorNumber() {return motorno; }

    /** this function is called in each timestep. It should perform robot-internal checks, 
	like space-internal collision detection, sensor resets/update etc.
	@param globalData structure that contains global data from the simulation environment
    */
    virtual void doInternalStuff(GlobalData& globalData) {}

    virtual bool collisionCallback(void *data, dGeomID o1, dGeomID o2) { return false;}

  protected:
    /** the main object of the robot, which is used for position and speed tracking */
    virtual Primitive* getMainPrimitive() const { return dummy; }

  protected:
    DummyPrimitive *dummy; 
    int sensorno;      //number of sensors
    int motorno;       // number of motors
    motor* motors;
  } ;

}

#endif
 
