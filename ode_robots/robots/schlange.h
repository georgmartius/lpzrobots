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
 *   Revision 1.15.4.8  2006-05-19 09:04:38  der
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

#include"primitive.h"
#include "joint.h"
#include "angularmotor.h"

#include "oderobot.h"
#include <selforg/configurable.h>

namespace lpzrobots {

typedef struct {
public:
  int    segmNumber;  //<  number of snake elements
  double segmLength;  //< length of one snake element
  double segmDia;     //<  diameter of a snake element
  double segmMass;    //<  mass of one snake element
  double motorPower;  //<  power of the motors / servos
  double sensorFactor;    //<  scale for sensors
  double frictionGround;  //< friction with ground
  double frictionJoint;   //< friction within joint
  double jointLimit;      //< maximal angle for the joints (M_PI/2 = 90 degree)
} SchlangeConf;


/**
 * This is a class, which models a snake like robot. 
 * It consists of a number of equal elements, each linked 
 * by a joint
 **/
class Schlange: public OdeRobot, public Configurable
{
protected:
  
  bool created;

  vector <Primitive*> objects;
  vector <Joint*> joints;
  vector <AngularMotor*> frictionmotors;
  SchlangeConf conf;

public:
  Schlange ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
	     const SchlangeConf& conf, const char* name);

  static SchlangeConf getDefaultConf(){
    SchlangeConf conf;
    conf.segmNumber = 10;    //  number of snake elements
    conf.segmLength = 0.8;   // length of one snake element
    conf.segmDia    = 0.2;   //  diameter of a snake element
    conf.segmMass   = 0.1;//0.4   //  mass of one snake element
    conf.motorPower = 1;    //  power of the servos
    conf.sensorFactor = 1;    //  scale for sensors
    conf.frictionGround = 1.0; // friction with ground
    conf.frictionJoint = 0.1; // friction within joint
    conf.jointLimit =  M_PI/4;
    return conf;
  }

  virtual ~Schlange();
	
 
  /** sets the pose of the vehicle
      @param pose desired 4x4 pose matrix
  */
  virtual void place(const osg::Matrix& pose);

  /// update all primitives and joints
  virtual void update();

  /**
   *This is the collision handling function for snake robots.
   *This overwrides the function collisionCallback of the class robot.
   *@param data
   *@param o1 first geometrical object, which has taken part in the collision
   *@param o2 second geometrical object, which has taken part in the collision
   *@return true if the collision was threated  by the robot, false if not
   **/
  virtual bool collisionCallback(void *data, dGeomID o1, dGeomID o2);	

  static void mycallback(void *data, dGeomID o1, dGeomID o2);

  virtual void doInternalStuff(const GlobalData& global);
	
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
  virtual int getSegmentsPosition(vector<Position> &poslist);

  /// returns the name of the object (with version number)
  virtual paramkey getName() const { return name; } 

  /** The list of all parameters with there value as allocated lists.
  */
  virtual paramlist getParamList() const;

  virtual paramval getParam(const paramkey& key) const;;

  virtual bool setParam(const paramkey& key, paramval val);

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

protected:

  /** creates vehicle at desired pose
      @param pose 4x4 pose matrix
  */
  virtual void create(const osg::Matrix& pose); 
  virtual void destroy();
};

}

#endif
