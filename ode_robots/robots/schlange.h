/************************************************************************/
/* schlange.h                                                                */
/* Abstract class for Snakes                                             */
/* @author Georg Martius                                                 */
/*                                                                        */
/************************************************************************/
/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
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
  double frictionJoint;   ///< friction within joint
  double frictionRatio;   ///< if != 1 then friction along the snake is the ratio fold
  double jointLimit;      ///< maximal angle for the joints (M_PI/4 = 45 degree)
  bool   useServoVel;     ///< if true the new Servos are used (only for schlangeservo)
  double velocity;        ///< maximal velocity of servos

  bool useSpaces;        ///< if true the snake is divided into subspaces (performance)

  std::string headColor;
  std::string bodyColor;
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
    conf.frictionJoint = 0.02; // friction within joint
    conf.frictionRatio = 1; // friction ratio
    conf.jointLimit =  M_PI/4;
    conf.useServoVel = false;
    conf.velocity    = 20;     // maximal velocity of servos
    conf.useSpaces   = true;
    conf.headColor   = "robot2";
    conf.bodyColor   = "robot1";
    return conf;
  }

  virtual ~Schlange();


  /** sets the pose of the vehicle
      @param pose desired 4x4 pose matrix
  */
  virtual void placeIntern(const osg::Matrix& pose);

  /// update all primitives and joints
  virtual void update();

  /**
   *Reads the actual motor commands from an array,
   *an sets all motors of the snake to this values.
   *It is an linear allocation.
   *@param motors pointer to the array, motor values are scaled to [-1,1]
   *@param motornumber length of the motor array
   **/
  virtual void setMotorsIntern( const double* motors, int motornumber ) = 0;

  /**
   *Writes the sensor values to an array in the memory.
   *@param sensors pointer to the array
   *@param sensornumber length of the sensor array
   *@return number of actually written sensors
   **/
  virtual int getSensorsIntern( double* sensors, int sensornumber ) = 0;

  /** returns number of sensors
   */
  virtual int getSensorNumberIntern() = 0;

  /** returns number of motors
   */
  virtual int getMotorNumberIntern() = 0;

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

  virtual std::vector<Primitive*> getAllPrimitives() const { return objects;}

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
  std::vector<OdeHandle> spaces;

  /** creates vehicle at desired pose
      @param pose 4x4 pose matrix
  */
  virtual void create(const osg::Matrix& pose);
  /**
     creates and initialised the segment with the given index
   */
  virtual Primitive* createSegment(int index, const OdeHandle& odeHandle);
  virtual void destroy();
};

}

#endif
