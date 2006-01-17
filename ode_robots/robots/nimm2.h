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
 *   Revision 1.17.4.7  2006-01-17 17:02:19  martius
 *   faster, stronger, more friction
 *
 *   Revision 1.17.4.6  2005/12/15 17:04:08  martius
 *   Primitives are not longer inherited from OSGPrimitive, moreover
 *   they aggregate them.
 *   Joint have better getter and setter
 *
 *   Revision 1.17.4.5  2005/12/14 15:37:09  martius
 *   robots are working with osg
 *
 *   Revision 1.17.4.4  2005/12/13 18:11:39  martius
 *   still trying to port robots
 *
 *   Revision 1.17.4.3  2005/11/16 11:26:52  martius
 *   moved to selforg
 *
 *   Revision 1.17.4.2  2005/11/15 12:29:26  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.17.4.1  2005/11/14 17:37:17  martius
 *   moved to selforg
 *
 *   Revision 1.17  2005/09/27 14:11:37  martius
 *   changed to use of Nimm2Conf
 *   IR sensors at front
 *
 *   Revision 1.16  2005/09/22 12:24:37  martius
 *   removed global variables
 *   OdeHandle and GlobalData are used instead
 *   sensor prepared
 *
 *   Revision 1.15  2005/09/22 11:22:15  martius
 *   removed global variables
 *   OdeHandle and GlobalData are used instead
 *   sensor prepared
 *
 *   Revision 1.14  2005/08/31 11:12:46  martius
 *   removed unused vars
 *
 *   Revision 1.13  2005/08/03 11:43:03  fhesse
 *   wheels moved out of center in cigarMode
 *
 *   Revision 1.12  2005/08/02 13:35:53  fhesse
 *   cigarMode added
 *
 *   Revision 1.11  2005/08/02 13:17:10  fhesse
 *   bumper added
 *
 *   Revision 1.10  2005/07/31 22:31:15  martius
 *   textures
 *
 *   Revision 1.9  2005/07/29 15:12:51  martius
 *   color modified
 *   spherical wheels are standart but adjustable
 *
 *   Revision 1.8  2005/07/26 17:03:25  martius
 *   resizeable
 *   forces and collisions fixed
 *
 *   Revision 1.7  2005/07/18 14:47:41  martius
 *   world, space, contactgroup are not pointers anymore.
 *
 *   Revision 1.6  2005/07/08 09:33:28  martius
 *   speed and force as optional constuctor parameter
 *
 *   Revision 1.5  2005/07/07 09:27:40  martius
 *   proper collision detection in car_space
 *
 *   Revision 1.4  2005/07/06 16:04:39  martius
 *   added collisioncallback to robot to perform smoother collisions of wheels with ground
 *
 *   Revision 1.3  2005/06/23 13:31:15  fhesse
 *   Vehicle changed to Nimm2
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __NIMM2_H
#define __NIMM2_H

#include "oderobot.h"
#include "raysensorbank.h"

#include "primitive.h"
#include "joint.h"

namespace lpzrobots {

typedef struct Bumper{
  Bumper() { trans = 0; bump = 0;}
  Primitive* trans;
  Primitive* bump;
} Bumper;

typedef struct {
  double size;
  double force;
  double speed;
  bool sphereWheels;
  bool bumper;
  bool cigarMode;
  bool irFront;
  bool irBack;
  bool irSide;
  bool singleMotor;
} Nimm2Conf;

/** Robot that looks like a Nimm 2 Bonbon :-)
    2 wheels and a cylinder like body   
*/
class Nimm2 : public OdeRobot{
public:
  
  Nimm2(const OdeHandle& odehandle, const OsgHandle& osgHandle, const Nimm2Conf& conf);

  static Nimm2Conf getDefaultConf(){
    Nimm2Conf conf;
    conf.size=1;
    conf.force=8;
    conf.speed=12;
    conf.sphereWheels=true;
    conf.bumper=false;
    conf.cigarMode=false;
    conf.irFront=false;
    conf.irBack=false;
    conf.irSide=false;
    conf.singleMotor=false;
    return conf;
  }

  virtual ~Nimm2(){};

  /**
   * updates the OSG nodes of the vehicle
   */
  virtual void update();

  /** sets the pose of the vehicle
      @params pose desired 4x4 pose matrix
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

  /** returns a vector with the positions of all segments of the robot
      @param poslist vector of positions (of all robot segments) 
      @return length of the list
  */
  virtual int getSegmentsPosition(vector<Position> &poslist);

  virtual bool collisionCallback(void *data, dGeomID o1, dGeomID o2);
  
    /** this function is called in each timestep. It should perform robot-internal checks, 
      like space-internal collision detection, sensor resets/update etc.
      @param GlobalData structure that contains global data from the simulation environment
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
  static void mycallback(void *data, dGeomID o1, dGeomID o2);

  Nimm2Conf conf;
  
  double length;  // chassis length
  double width;  // chassis width
  double height;   // chassis height
  double radius;  // wheel radius
  double wheelthickness; // thickness of the wheels  
  double cmass;    // chassis mass
  double wmass;    // wheel mass
  int sensorno;      //number of sensors
  int motorno;       // number of motors

  bool created;      // true if robot was created
  double max_force; 

  Primitive* object[3];  // 1 cylinder, 2 wheels
  double  wheeloffset; // offset from center when in cigarMode
  int number_bumpers;  // number of bumpers (1 -> bumpers at one side, 2 -> bumpers at 2 sides)
  Bumper bumper[2]; 
  Hinge2Joint* joint[2]; // joints between cylinder and each wheel

  RaySensorBank irSensorBank; // a collection of ir sensors
};

}

#endif
