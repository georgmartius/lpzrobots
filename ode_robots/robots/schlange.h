/************************************************************************/
/* schlange.h						        	*/
/* Abstract class for Snakes                             		*/
/* @author Georg Martius 						*/
/*									*/
/************************************************************************/
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
 *   Revision 1.28  2011-05-30 13:56:42  martius
 *   clean up: moved old code to oldstuff
 *   configable changed: notifyOnChanges is now used
 *    getParam,setParam, getParamList is not to be overloaded anymore
 *
 *   Revision 1.27  2010/10/20 13:17:28  martius
 *   comment
 *
 *   Revision 1.26  2010/09/30 17:12:29  martius
 *   added anisotrop friction to schlange
 *
 *   Revision 1.25  2010/01/26 09:55:26  martius
 *   new collision model
 *
 *   Revision 1.24  2009/05/11 15:44:30  martius
 *   new velocity servos used
 *
 *   Revision 1.23  2007/11/07 13:21:16  martius
 *   doInternal stuff changed signature
 *
 *   Revision 1.22  2007/07/03 13:05:23  martius
 *   new servo constants
 *
 *   Revision 1.21  2007/01/26 12:05:04  martius
 *   servos combinied into OneAxisServo
 *
 *   Revision 1.20  2006/12/21 11:43:05  martius
 *   commenting style for doxygen //< -> ///<
 *   new sensors for spherical robots
 *
 *   Revision 1.19  2006/09/20 12:56:16  martius
 *   Snakes have CreateSegment
 *
 *   Revision 1.18  2006/07/20 17:19:44  martius
 *   removed using namespace std from matrix.h
 *
 *   Revision 1.17  2006/07/14 13:52:01  der
 *   setheadcolor
 *
 *   Revision 1.15.4.9  2006/06/25 16:57:15  martius
 *   abstractrobot is configureable
 *   name and revision
 *
 *   Revision 1.15.4.8  2006/05/19 09:04:38  der
 *   -setTexture and setHeadTexture added
 *   -uses now whitemetal texture
 *
 *   Revision 1.15.4.7  2006/03/30 12:34:56  martius
 *   documentation updated
 *
 *   Revision 1.15.4.6  2006/03/29 15:08:54  martius
 *   getMainPrimitive is public now
 *
 *   Revision 1.15.4.5  2006/02/23 18:05:04  martius
 *   friction with angularmotor
 *
 *   Revision 1.15.4.4  2006/02/01 18:33:40  martius
 *   use Axis type for Joint axis. very important, since otherwise Vec3 * pose is not the right direction vector anymore
 *
 *   Revision 1.15.4.3  2005/12/30 22:53:13  martius
 *   removed parentspace!
 *
 *   Revision 1.15.4.2  2005/12/29 16:45:46  martius
 *   does not inherit from Roboter
 *   moved to osg
 *
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __SCHLANGE_H
#define __SCHLANGE_H

#include<vector>
#include<assert.h>

#include "primitive.h"
#include "joint.h"
#include "angularmotor.h"
#include "pid.h"

#include "oderobot.h"
#include <selforg/configurable.h>

namespace lpzrobots {

typedef struct {
public:
  int    segmNumber;  ///<  number of snake elements
  double segmLength;  ///< length of one snake element
  double segmDia;     ///<  diameter of a snake element
  double segmMass;    ///<  mass of one snake element
  double motorPower;  ///<  power of the motors / servos
  double sensorFactor;    ///<  scale for sensors
  double frictionGround;  ///< friction with ground
  double frictionJoint;   ///< friction within joint
  double frictionRatio;   ///< if != 1 then friction along the snake is the ratio fold 
  double jointLimit;      ///< maximal angle for the joints (M_PI/4 = 45 degree)
  bool   useServoVel;     ///< if true the new Servos are used (only for schlangeservo)
  double velocity;        ///< maximal velocity of servos
} SchlangeConf;


/**
 * This is a class, which models a snake like robot. 
 * It consists of a number of equal elements, each linked 
 * by a joint
 **/
class Schlange: public OdeRobot
{
protected:
  
  bool created;

  std::vector <Primitive*> objects;
  std::vector <Joint*> joints;
  std::vector <AngularMotor*> frictionmotors;
  SchlangeConf conf;

public:
  Schlange ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
	     const SchlangeConf& conf, const std::string& name, const std::string& revision);

  static SchlangeConf getDefaultConf(){
    SchlangeConf conf;
    conf.segmNumber = 10;    //  number of snake elements
    conf.segmLength = 0.8;   // length of one snake element
    conf.segmDia    = 0.2;   //  diameter of a snake element
    conf.segmMass   = 0.1;//0.4   //  mass of one snake element
    conf.motorPower = 1;    //  power of the servos
    conf.sensorFactor = 1;    //  scale for sensors
    conf.frictionGround = 1.0; // friction with ground
    conf.frictionJoint = 0.02; // friction within joint
    conf.frictionRatio = 1; // friction ratio
    conf.jointLimit =  M_PI/4;
    conf.useServoVel = false;
    conf.velocity    = 20;     // maximal velocity of servos
    return conf;
  }

  virtual ~Schlange();
	
 
  /** sets the pose of the vehicle
      @param pose desired 4x4 pose matrix
  */
  virtual void place(const osg::Matrix& pose);

  /// update all primitives and joints
  virtual void update();

  virtual void doInternalStuff(GlobalData& global);
	
  /**
   *Reads the actual motor commands from an array, 
   *an sets all motors of the snake to this values.
   *It is an linear allocation.
   *@param motors pointer to the array, motor values are scaled to [-1,1] 
   *@param motornumber length of the motor array
   **/
  virtual void setMotors ( const motor* motors, int motornumber ) = 0;

  /**
   *Writes the sensor values to an array in the memory.
   *@param sensors pointer to the array
   *@param sensornumber length of the sensor array
   *@return number of actually written sensors
   **/
  virtual int getSensors ( sensor* sensors, int sensornumber ) = 0;
	
  /** returns number of sensors
   */
  virtual int getSensorNumber() = 0;

  /** returns number of motors
   */
  virtual int getMotorNumber() = 0;

  /** returns a vector with the positions of all segments of the robot
      @param poslist vector of positions (of all robot segments) 
      @return length of the list
  */
  virtual int getSegmentsPosition(std::vector<Position> &poslist);


  /******** CONFIGURABLE ***********/
  virtual void notifyOnChange(const paramkey& key);

  /** the main object of the robot, which is used for position and speed tracking */
  virtual Primitive* getMainPrimitive() const {
    if(!objects.empty()){
      //      int half = objects.size()/2;
      //      return (objects[half]);
      return (objects[0]);
    }else return 0;
  }

  /** sets a texture to the body of the snake
   * note: the head texture of the snake is set by
   * this method too!
   */
  virtual void setTexture(const std::string& filename);
  
  /** sets a texture to the head of the snake
   */
  virtual void setHeadTexture(const std::string& filename);

  /**
   * sets the color of the head element
   */
  virtual void setHeadColor(const Color& color);


protected:

  /** creates vehicle at desired pose
      @param pose 4x4 pose matrix
  */
  virtual void create(const osg::Matrix& pose); 
  /**
     creates and initialised the segment with the given index
   */
  virtual Primitive* createSegment(int index); 
  virtual void destroy();
};

}

#endif
