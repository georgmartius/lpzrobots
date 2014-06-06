/************************************************************************/
/* originally from:                                                     */
/* schlange.h                                                                */
/* Abstract class for Snakes                                             */
/* @author Georg Martius                                                 */
/************************************************************************/
/* here:                                                                */
/* defaultCaterpillar.h                                                 */
/* Abstract class for Caterpillars                                             */
/* @author Frank Guettler                                                 */
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
#ifndef __DEFAULTCATERPILLAR_H
#define __DEFAULTCATERPILLAR_H

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
  int    segmNumber;  ///<  number of snake elements
  double segmLength;  ///< length of one snake element
  double segmDia;     ///<  diameter of a snake element
  double segmMass;    ///<  mass of one snake element
  double motorPower;  ///<  power of the motors / servos
  double sensorFactor;    ///<  scale for sensors
  double frictionGround;  ///< friction with ground
  double frictionJoint;   ///< friction within joint
  double jointLimit;      ///< maximal angle for the joints (M_PI/2 = 90 degree)
  int firstJoint;        ///< first joint type to use: 0=sliderJoint, 1=universalJoint
} CaterPillarConf;


/**
 * This is a class, which models a snake like robot.
 * It consists of a number of equal elements, each linked
 * by a joint
 **/
class DefaultCaterPillar: public OdeRobot
{
protected:

  bool created;

  std::vector <AngularMotor*> frictionmotors;
  CaterPillarConf conf;

public:
  DefaultCaterPillar ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
             const CaterPillarConf& conf, const std::string& name, const std::string& revision);

  static CaterPillarConf getDefaultConf(){
    CaterPillarConf conf;
    conf.segmNumber = 6;    //  number of snake elements
    conf.segmLength = 0.4;   // length of one snake element
    conf.segmDia    = 0.2;   //  diameter of a snake element
    conf.segmMass   = 0.4;   //  mass of one snake element
    conf.motorPower = 1;    //  power of the servos
    conf.sensorFactor = 1;    //  scale for sensors
    conf.frictionGround = 1.0; // friction with ground
    conf.frictionJoint = 0.1; // friction within joint
    conf.jointLimit =  M_PI/8;
    conf.firstJoint=1;
    return conf;
  }

  virtual ~DefaultCaterPillar();


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
  virtual int getSensorsIntern( sensor* sensors, int sensornumber ) = 0;

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
protected:

  /** creates vehicle at desired pose
      @param pose 4x4 pose matrix
  */
  virtual void create(const osg::Matrix& pose);
  virtual void destroy();
};

}

#endif
