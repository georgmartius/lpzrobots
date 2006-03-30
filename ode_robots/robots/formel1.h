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
 *   Revision 1.2.4.5  2006-03-30 12:34:56  martius
 *   documentation updated
 *
 *   Revision 1.2.4.4  2006/01/12 14:47:47  martius
 *   just taken from nimm4
 *
 *   Revision 1.2.4.3  2005/11/16 11:26:52  martius
 *   moved to selforg
 *
 *   Revision 1.2.4.2  2005/11/15 12:29:26  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.2.4.1  2005/11/14 17:37:17  martius
 *   moved to selforg
 *
 *   Revision 1.2  2005/09/22 12:24:37  martius
 *   removed global variables
 *   OdeHandle and GlobalData are used instead
 *   sensor prepared
 *
 *   Revision 1.1  2005/08/09 10:44:54  robot1
 *   first vehicle for formel1 simulation
 *
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __FORMEL1_H
#define __FORMEL1_H

#include "oderobot.h"

namespace lpzrobots {

  class Primitive;
  class Joint;

  /** Robot that looks like a Nimm 2 Bonbon :-)
      4 wheels and a capsule like body   
  */
  class Formel1 : public OdeRobot{
  public:
  
    Formel1(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
	  double size=1, double force=3, double speed=15, bool sphereWheels=true);

    virtual ~Formel1(){};

    /**
     * updates the OSG nodes of the vehicle
     */
    virtual void update();


    /** sets the pose of the vehicle
	@param pose desired pose matrix
    */
    virtual void place(const osg::Matrix& pose);

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

    /** checks for internal collisions and treats them. 
     *  In case of a treatment return true (collision will be ignored by other objects 
     *  and the default routine)  else false (collision is passed to other objects and 
     *  (if not treated) to the default routine).
     */
    virtual bool collisionCallback(void *data, dGeomID o1, dGeomID o2);

    /** this function is called in each timestep. It should perform robot-internal checks, 
	like space-internal collision detection, sensor resets/update etc.
	@param globalData structure that contains global data from the simulation environment
    */
    virtual void doInternalStuff(const GlobalData& globalData);


  protected:
    /** the main object of the robot, which is used for position and speed tracking */
    virtual Primitive* getMainPrimitive() const { return object[0]; }

    /** creates vehicle at desired pose
	@param pose 4x4 pose matrix
    */
    virtual void create(const osg::Matrix& pose); 

    /** destroys vehicle and space
     */
    virtual void destroy();

    /** additional things for collision handling can be done here
     */
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

    bool created;      // true if robot was created

    Primitive* object[5];  // 1 capsule, 4 wheels
    Hinge2Joint* joint[4]; // joints between cylinder and each wheel

  };

}

#endif
