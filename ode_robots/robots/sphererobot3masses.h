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
 * Spherical Robot inspired by Julius Popp.                                *
 *                                                                         *
 *   $Log$
 *   Revision 1.8  2006-12-01 16:20:40  martius
 *   *** empty log message ***
 *
 *   Revision 1.7  2006/11/17 13:44:43  martius
 *   corrected z-axes sensor problem
 *   there are two sensors for this situation
 *
 *   Revision 1.6  2006/09/21 22:09:58  martius
 *   collision for mesh
 *
 *   Revision 1.5  2006/09/21 16:17:18  der
 *   *** empty log message ***
 *
 *   Revision 1.4  2006/08/04 15:07:27  martius
 *   documentation
 *
 *   Revision 1.3  2006/07/20 17:19:45  martius
 *   removed using namespace std from matrix.h
 *
 *   Revision 1.2  2006/07/14 12:23:42  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.4  2006/06/25 16:57:17  martius
 *   abstractrobot is configureable
 *   name and revision
 *
 *   Revision 1.1.2.3  2006/03/30 12:34:57  martius
 *   documentation updated
 *
 *   Revision 1.1.2.2  2006/01/10 17:15:44  martius
 *   removed wrong comment
 *
 *   Revision 1.1.2.1  2006/01/10 17:15:16  martius
 *   was sphererobotarms
 *   moved to osg
 *
 *   Revision 1.10.4.3  2005/11/16 11:26:53  martius
 *   moved to selforg
 *
 *   Revision 1.10.4.2  2005/11/15 12:29:27  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.10.4.1  2005/11/14 17:37:18  martius
 *   moved to selforg
 *
 *   Revision 1.10  2005/11/09 13:27:07  martius
 *   irsensorrange
 *
 *   Revision 1.9  2005/11/07 17:04:56  martius
 *   irsensorscale added
 *
 *   Revision 1.8  2005/11/04 14:45:18  martius
 *   GPL added
 *
 *                                                                 *
 ***************************************************************************/

#ifndef __SPHEREROBOT3MASSES_H
#define __SPHEREROBOT3MASSES_H

#include "primitive.h"
#include "joint.h"
#include "sliderservo.h"
#include "oderobot.h"
#include "raysensorbank.h"

namespace lpzrobots {

  /// configuration object for the Sphererobot3Masses robot.
typedef struct {
public:
  double diameter;
  double spheremass;
  double pendulardiameter; //< automatically set
  double pendularmass;
  double pendularrange;
  bool motorsensor;        //< motor values as sensors
  bool worldZaxissensor;   //< world cooridinate of z axis
  bool axisZsensor;        //< z-coordinate of all internal axes
  bool axisXYZsensor;      //< entire rotation matrix
  bool irAxis1;
  bool irAxis2;
  bool irAxis3;
  bool drawIRs;
  double irsensorscale; //< range of the ir sensors in units of diameter
  double irCharacter;   //< characteristics of sensor (\f[ x^c \f] where x is the range-distance)
} Sphererobot3MassesConf;

/**
   A spherical robot with 3 internal masses, which can slide on their orthogonal axes.
   This robot was inspired by Julius Popp (http://sphericalrobots.com)
*/
class Sphererobot3Masses : public OdeRobot
{
public:
  /// enum for the objects of the robot
  typedef enum objects { Base, Pendular1, Pendular2, Pendular3, Last } ;

protected:
  static const int servono=3;
  unsigned int numberaxis;

  Primitive* object[Last]; 
  SliderServo* servo[servono];
  SliderJoint* joint[servono];
  OSGPrimitive* axis[servono];

  Sphererobot3MassesConf conf;
  RaySensorBank irSensorBank; //< a collection of ir sensors  
  double transparency;
  bool created;

public:

  /**
   *constructor
   **/ 
  Sphererobot3Masses ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
		       const Sphererobot3MassesConf& conf, const std::string& name, double transparency=0.5 );

protected:
  /**
   *constructor for children
   **/ 
  Sphererobot3Masses ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
		       const Sphererobot3MassesConf& conf, 
		       const std::string& name, const std::string& revision, double transparency);
  /// initialises some internal variables
  void init();
public:  
  virtual ~Sphererobot3Masses();

	
  /// default configuration
  static Sphererobot3MassesConf getDefaultConf(){
    Sphererobot3MassesConf c;
    c.diameter     = 1;
    c.spheremass   = .3;// 0.1
    c.pendularmass  = 1.0;
    c.pendularrange  = 0.25; // range of the slider from center in multiple of diameter [-range,range]
    c.axisZsensor = true;
    c.axisXYZsensor = false;  
    c.worldZaxissensor = false;
    c.motorsensor = false;  
    c.irAxis1=false;
    c.irAxis2=false;
    c.irAxis3=false;
    c.drawIRs=true;
    c.irsensorscale=1.5;
    c.irCharacter=1;  
    return c;
  }

  virtual void update();

  virtual void place(const osg::Matrix& pose);
  
  virtual bool collisionCallback(void *data, dGeomID o1, dGeomID o2);

  virtual void doInternalStuff(const GlobalData& globalData);
	
  virtual int getSensors ( sensor* sensors, int sensornumber );
	
  virtual void setMotors ( const motor* motors, int motornumber );
	
  virtual int getMotorNumber();
  
  virtual int getSensorNumber();
	
 
  /** the main object of the robot, which is used for position and speed tracking */
  virtual Primitive* getMainPrimitive() const { return object[Base]; }

protected:

  virtual void create(const osg::Matrix& pose); 
  virtual void destroy(); 


};

}

#endif
