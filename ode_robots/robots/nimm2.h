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
 *   Revision 1.31  2011-08-04 16:43:53  martius
 *   guilogger is positioned beside simulation window (can still be improved)
 *   ctrl-h can be used to move observed agent to 0,0,0 position
 *
 *   Revision 1.30  2011/06/03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.29  2011/04/28 09:43:58  martius
 *   added configuration possiblity
 *
 *   Revision 1.28  2007/11/07 13:21:16  martius
 *   doInternal stuff changed signature
 *
 *   Revision 1.27  2007/09/27 16:01:28  der
 *   added to nimm2 the box version
 *
 *   Revision 1.26  2007/05/07 21:12:19  robot3
 *   added experimental force sensors
 *
 *   Revision 1.25  2007/03/06 10:11:04  fhesse
 *   food removed
 *
 *   Revision 1.24  2007/03/05 10:49:32  fhesse
 *   food at position x=0, y=0 added
 *   motorcommand y set to zero when near food (controller doesn't know)
 *   after eating_time food is empty and motorcommand executed again
 *
 *   Revision 1.23  2007/02/23 15:14:17  martius
 *   *** empty log message ***
 *
 *   Revision 1.22  2006/12/21 11:43:05  martius
 *   commenting style for doxygen //< -> ///<
 *   new sensors for spherical robots
 *
 *   Revision 1.21  2006/12/11 18:24:36  martius
 *   memory freed
 *
 *   Revision 1.20  2006/09/21 22:09:58  martius
 *   collision for mesh
 *
 *   Revision 1.19  2006/07/20 17:19:44  martius
 *   removed using namespace std from matrix.h
 *
 *   Revision 1.18  2006/07/14 12:23:40  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.17.4.10  2006/06/25 16:57:14  martius
 *   abstractrobot is configureable
 *   name and revision
 *
 *   Revision 1.17.4.9  2006/03/30 12:34:56  martius
 *   documentation updated
 *
 *   Revision 1.17.4.8  2006/01/31 15:36:14  martius
 *   irRange in config
 *
 *   Revision 1.17.4.7  2006/01/17 17:02:19  martius
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
#include <selforg/inspectable.h>


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
  double wheelSize; ///< size of the wheels in body diameters
  std::string wheelTexture;
  bool bumper;
  bool cigarMode;
  double cigarLength;
  bool irFront;
  bool irBack;
  bool irSide;
  double irRange;
  bool singleMotor;
  bool visForce;
  bool boxMode;
  double  boxWidth;
} Nimm2Conf;

/** Robot that looks like a Nimm 2 Bonbon :-)
    2 wheels and a cylinder like body
*/
class Nimm2 : public OdeRobot /*, public Inspectable*/ {
public:

  Nimm2(const OdeHandle& odehandle, const OsgHandle& osgHandle,
	const Nimm2Conf& conf, const std::string& name);

  static Nimm2Conf getDefaultConf(){
    Nimm2Conf conf;
    conf.size=1;
    conf.force=5;
    conf.speed=12;
    conf.sphereWheels=true;
    conf.wheelSize=1;
    conf.bumper=false;
    conf.cigarMode=false;
    conf.cigarLength=2.0;
    conf.irFront=false;
    conf.irBack=false;
    conf.irSide=false;
    conf.irRange=3;
    conf.singleMotor=false;
    conf.visForce=false;
    conf.boxMode=false;
    conf.boxWidth=1.0;
    conf.wheelTexture="Images/tire.rgb";
    return conf;
  }

  virtual ~Nimm2();

  /**
   * updates the OSG nodes of the vehicle
   */
  virtual void update();

  /** sets the pose of the vehicle
      @param pose desired 4x4 pose matrix
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
  virtual int getSegmentsPosition(std::vector<Position> &poslist);

  virtual bool collisionCallback(void *data, dGeomID o1, dGeomID o2);

    /** this function is called in each timestep. It should perform robot-internal checks,
      like space-internal collision detection, sensor resets/update etc.
      @param globalData structure that contains global data from the simulation environment
   */
  virtual void doInternalStuff(GlobalData& globalData);

	virtual double& getSumForce() { return sumForce; }

	virtual double& getContactPoints() { return contactPoints; }

protected:

  double contactPoints;


  /** creates vehicle at desired pose
      @param pose 4x4 pose matrix
  */
  virtual void create(const osg::Matrix& pose);



  /** destroys vehicle and space
   */
  virtual void destroy();
  static void mycallback(void *data, dGeomID o1, dGeomID o2);

	/**
	 * Inspectable interface
	 */
	/*
	virtual std::list<iparamkey> getInternalParamNames() const  { return std::list<iparamkey>(); }

	virtual std::list<iparamval> getInternalParams() const { return std::list<iparamval>(); }*/
	/*
	virtual std::list<Inspectable::iparamkey> getInternalParamNames() const;

	virtual std::list<Inspectable::iparamval> getInternalParams() const;
	*/

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

  double  wheeloffset; // offset from center when in cigarMode
  int number_bumpers;  // number of bumpers (1 -> bumpers at one side, 2 -> bumpers at 2 sides)
  Bumper bumper[2];

  RaySensorBank irSensorBank; // a collection of ir sensors

	bool visForce; // decides if contact force is made visible in guilogger
	double sumForce; // stores the contact force made by collisions with external objects

};

}

#endif
